#include "MAVFTPFileFormats.h"
#include "MAVFTPProtocol.h"

#include <QByteArray>
#include <QCoreApplication>
#include <QElapsedTimer>
#include <QHostAddress>
#include <QList>
#include <QMap>
#include <QPair>
#include <QString>
#include <QTextStream>
#include <QThread>
#include <QUdpSocket>
#include <QVariant>
#include <QtGlobal>

#include <cmath>
#include <cstring>

namespace {

const uint8_t kGroundStationSystemId = 255;
const uint8_t kGroundStationComponentId = MAV_COMP_ID_MISSIONPLANNER;
const int kDefaultUdpPort = 5760;
const int kConnectTimeoutMs = 90000;
const int kRequestTimeoutMs = 15000;

void logInfo(const QString& message)
{
    QTextStream(stdout) << message << '\n';
}

void logError(const QString& message)
{
    QTextStream(stderr) << message << '\n';
}

quint32 readUInt32(const QByteArray& data, int offset)
{
    const uchar* bytes = reinterpret_cast<const uchar*>(data.constData() + offset);
    return static_cast<quint32>(bytes[0]) |
            (static_cast<quint32>(bytes[1]) << 8) |
            (static_cast<quint32>(bytes[2]) << 16) |
            (static_cast<quint32>(bytes[3]) << 24);
}

bool nearlyEqual(double a, double b)
{
    return std::fabs(a - b) < 0.001;
}

double variantToDouble(const QVariant& value)
{
    return value.type() == QVariant::Char ? value.toChar().toLatin1() : value.toDouble();
}

QVariant uploadVariantFor(const MAVFTPFileFormats::ParameterValue& parameter, double value)
{
    switch (parameter.packedType) {
    case 1:
        return QVariant(QChar(static_cast<ushort>(static_cast<quint8>(static_cast<int>(value)))));
    case 2:
    case 3:
        return QVariant(static_cast<int>(value));
    case 4:
        return QVariant(value);
    default:
        return QVariant();
    }
}

class SitlFtpClient
{
public:
    bool connectToVehicle(int port, QString* errorString);
    bool downloadFile(const QString& path, QByteArray* data, bool* sawMultipleChunks, bool* sawEofNack, QString* errorString);
    bool uploadFile(const QString& path, const QByteArray& data, bool* sawMultipleChunks, QString* errorString);
    bool expectMissingFileNack(QString* errorString);

private:
    bool sendRequest(uint8_t session, uint8_t opcode, uint8_t size, quint32 offset, const QByteArray& data,
                     MAVFTPProtocol::Packet* response, QString* errorString);
    bool waitForResponse(quint16 requestSequence, uint8_t requestOpcode, MAVFTPProtocol::Packet* response,
                         QString* errorString);
    bool terminateSession(uint8_t session, QString* errorString);
    bool sendMavlinkMessage(const mavlink_message_t& message, QString* errorString);
    bool parseIncomingDatagrams(MAVFTPProtocol::Packet* response, bool* gotResponse, QString* errorString);
    bool isResponseForUs(const mavlink_message_t& message) const;

    QUdpSocket _socket;
    QHostAddress _vehicleAddress;
    quint16 _vehiclePort = 0;
    uint8_t _targetSystem = 0;
    uint8_t _targetComponent = 0;
    quint16 _sequence = 0;
    mavlink_status_t _mavlinkStatus {};
};

bool SitlFtpClient::connectToVehicle(int port, QString* errorString)
{
    if (!_socket.bind(QHostAddress::LocalHost, static_cast<quint16>(port), QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint)) {
        *errorString = QStringLiteral("failed to bind UDP port %1: %2").arg(port).arg(_socket.errorString());
        return false;
    }

    logInfo(QStringLiteral("Waiting for ArduPilot heartbeat on UDP port %1").arg(port));
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < kConnectTimeoutMs) {
        if (!_socket.waitForReadyRead(1000)) {
            continue;
        }

        while (_socket.hasPendingDatagrams()) {
            QByteArray datagram;
            datagram.resize(static_cast<int>(_socket.pendingDatagramSize()));
            QHostAddress sender;
            quint16 senderPort = 0;
            _socket.readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

            for (int i = 0; i < datagram.size(); i++) {
                mavlink_message_t message;
                if (!mavlink_parse_char(MAVLINK_COMM_0, static_cast<uint8_t>(datagram.at(i)), &message, &_mavlinkStatus)) {
                    continue;
                }

                if (message.msgid != MAVLINK_MSG_ID_HEARTBEAT) {
                    continue;
                }

                mavlink_heartbeat_t heartbeat;
                mavlink_msg_heartbeat_decode(&message, &heartbeat);
                if (heartbeat.autopilot != MAV_AUTOPILOT_ARDUPILOTMEGA) {
                    continue;
                }

                _vehicleAddress = sender;
                _vehiclePort = senderPort;
                _targetSystem = message.sysid;
                _targetComponent = message.compid;
                logInfo(QStringLiteral("Connected to ArduPilot sysid=%1 compid=%2 at %3:%4")
                        .arg(_targetSystem)
                        .arg(_targetComponent)
                        .arg(_vehicleAddress.toString())
                        .arg(_vehiclePort));
                return true;
            }
        }
    }

    *errorString = QStringLiteral("timed out waiting for ArduPilot heartbeat");
    return false;
}

bool SitlFtpClient::sendRequest(uint8_t session, uint8_t opcode, uint8_t size, quint32 offset, const QByteArray& data,
                                MAVFTPProtocol::Packet* response, QString* errorString)
{
    const quint16 requestSequence = _sequence;
    _sequence = static_cast<quint16>(_sequence + 2);

    uint8_t payload[MAVFTPProtocol::PayloadLength];
    if (!MAVFTPProtocol::encodePayload(requestSequence, session, opcode, size, offset, data,
                                       payload, MAVFTPProtocol::PayloadLength, errorString)) {
        return false;
    }

    mavlink_message_t message;
    mavlink_msg_file_transfer_protocol_pack(kGroundStationSystemId,
                                            kGroundStationComponentId,
                                            &message,
                                            0,
                                            _targetSystem,
                                            _targetComponent,
                                            payload);
    if (!sendMavlinkMessage(message, errorString)) {
        return false;
    }

    return waitForResponse(requestSequence, opcode, response, errorString);
}

bool SitlFtpClient::waitForResponse(quint16 requestSequence, uint8_t requestOpcode, MAVFTPProtocol::Packet* response,
                                    QString* errorString)
{
    QElapsedTimer timer;
    timer.start();
    while (timer.elapsed() < kRequestTimeoutMs) {
        const int remaining = qMax(1, kRequestTimeoutMs - static_cast<int>(timer.elapsed()));
        if (!_socket.waitForReadyRead(qMin(remaining, 1000))) {
            continue;
        }

        bool gotResponse = false;
        MAVFTPProtocol::Packet packet;
        if (!parseIncomingDatagrams(&packet, &gotResponse, errorString)) {
            return false;
        }
        if (!gotResponse) {
            continue;
        }
        if (packet.sequence != MAVFTPProtocol::expectedResponseSequence(requestSequence)) {
            continue;
        }
        if (packet.requestOpcode != requestOpcode) {
            continue;
        }

        *response = packet;
        return true;
    }

    *errorString = QStringLiteral("timed out waiting for MAVFTP response to opcode %1").arg(requestOpcode);
    return false;
}

bool SitlFtpClient::downloadFile(const QString& path, QByteArray* data, bool* sawMultipleChunks, bool* sawEofNack,
                                 QString* errorString)
{
    data->clear();
    *sawMultipleChunks = false;
    *sawEofNack = false;

    const QByteArray pathBytes = path.toLatin1();
    MAVFTPProtocol::Packet response;
    if (!sendRequest(0, MAVFTPProtocol::OpOpenFileRO, static_cast<uint8_t>(pathBytes.size()), 0, pathBytes, &response, errorString)) {
        return false;
    }
    if (response.opcode != MAVFTPProtocol::OpAck || response.data.size() < 4) {
        *errorString = QStringLiteral("open read failed for %1: opcode=%2 error=%3")
                .arg(path)
                .arg(response.opcode)
                .arg(MAVFTPProtocol::errorString(MAVFTPProtocol::responseErrorCode(response)));
        return false;
    }

    const uint8_t session = response.session;
    const quint32 advertisedSize = readUInt32(response.data, 0);
    logInfo(QStringLiteral("Opened %1, advertised size %2").arg(path).arg(advertisedSize));

    quint32 offset = 0;
    int chunks = 0;
    while (true) {
        if (!sendRequest(session, MAVFTPProtocol::OpReadFile, MAVFTPProtocol::MaxDataLength, offset, QByteArray(), &response, errorString)) {
            return false;
        }
        if (response.opcode == MAVFTPProtocol::OpNack) {
            if (MAVFTPProtocol::responseErrorCode(response) == MAVFTPProtocol::ErrEndOfFile) {
                *sawEofNack = true;
                break;
            }
            *errorString = QStringLiteral("read failed for %1 at offset %2: %3")
                    .arg(path)
                    .arg(offset)
                    .arg(MAVFTPProtocol::errorString(MAVFTPProtocol::responseErrorCode(response)));
            return false;
        }
        if (response.opcode != MAVFTPProtocol::OpAck || response.offset != offset) {
            *errorString = QStringLiteral("unexpected read response for %1: opcode=%2 offset=%3 expected=%4")
                    .arg(path)
                    .arg(response.opcode)
                    .arg(response.offset)
                    .arg(offset);
            return false;
        }
        data->append(response.data);
        offset += response.data.size();
        chunks++;
        if (response.data.size() < MAVFTPProtocol::MaxDataLength) {
            break;
        }
    }

    if (!*sawEofNack) {
        if (!sendRequest(session, MAVFTPProtocol::OpReadFile, MAVFTPProtocol::MaxDataLength, offset, QByteArray(), &response, errorString)) {
            return false;
        }
        if (response.opcode != MAVFTPProtocol::OpNack ||
                MAVFTPProtocol::responseErrorCode(response) != MAVFTPProtocol::ErrEndOfFile) {
            *errorString = QStringLiteral("expected EOF NAK for %1 at offset %2").arg(path).arg(offset);
            return false;
        }
        *sawEofNack = true;
    }

    if (!terminateSession(session, errorString)) {
        return false;
    }

    *sawMultipleChunks = chunks > 1;
    logInfo(QStringLiteral("Downloaded %1 bytes from %2 in %3 chunks").arg(data->size()).arg(path).arg(chunks));
    return true;
}

bool SitlFtpClient::uploadFile(const QString& path, const QByteArray& data, bool* sawMultipleChunks, QString* errorString)
{
    *sawMultipleChunks = false;

    const QByteArray pathBytes = path.toLatin1();
    MAVFTPProtocol::Packet response;
    if (!sendRequest(0, MAVFTPProtocol::OpCreateFile, static_cast<uint8_t>(pathBytes.size()), 0, pathBytes, &response, errorString)) {
        return false;
    }
    if (response.opcode != MAVFTPProtocol::OpAck) {
        *errorString = QStringLiteral("create failed for %1: %2")
                .arg(path)
                .arg(MAVFTPProtocol::errorString(MAVFTPProtocol::responseErrorCode(response)));
        return false;
    }

    const uint8_t session = response.session;
    quint32 offset = 0;
    int chunks = 0;
    while (offset < static_cast<quint32>(data.size())) {
        const int writeSize = qMin(static_cast<int>(MAVFTPProtocol::MaxDataLength), data.size() - static_cast<int>(offset));
        const QByteArray chunk = data.mid(static_cast<int>(offset), writeSize);
        if (!sendRequest(session, MAVFTPProtocol::OpWriteFile, static_cast<uint8_t>(chunk.size()), offset, chunk, &response, errorString)) {
            return false;
        }
        if (response.opcode != MAVFTPProtocol::OpAck || response.offset != offset) {
            *errorString = QStringLiteral("write failed for %1 at offset %2: opcode=%3 error=%4")
                    .arg(path)
                    .arg(offset)
                    .arg(response.opcode)
                    .arg(MAVFTPProtocol::errorString(MAVFTPProtocol::responseErrorCode(response)));
            return false;
        }

        offset += chunk.size();
        chunks++;
    }

    if (!terminateSession(session, errorString)) {
        return false;
    }

    *sawMultipleChunks = chunks > 1;
    logInfo(QStringLiteral("Uploaded %1 bytes to %2 in %3 chunks").arg(data.size()).arg(path).arg(chunks));
    return true;
}

bool SitlFtpClient::expectMissingFileNack(QString* errorString)
{
    MAVFTPProtocol::Packet response;
    const QByteArray path = "@PARAM/does-not-exist.pck";
    if (!sendRequest(0, MAVFTPProtocol::OpOpenFileRO, static_cast<uint8_t>(path.size()), 0, path, &response, errorString)) {
        return false;
    }
    if (response.opcode != MAVFTPProtocol::OpNack ||
            MAVFTPProtocol::responseErrorCode(response) != MAVFTPProtocol::ErrFileNotFound) {
        *errorString = QStringLiteral("expected FileNotFound NAK for missing file, got opcode=%1 error=%2")
                .arg(response.opcode)
                .arg(MAVFTPProtocol::errorString(MAVFTPProtocol::responseErrorCode(response)));
        return false;
    }
    logInfo(QStringLiteral("Missing-file NAK path verified"));
    return true;
}

bool SitlFtpClient::terminateSession(uint8_t session, QString* errorString)
{
    MAVFTPProtocol::Packet response;
    if (!sendRequest(session, MAVFTPProtocol::OpTerminateSession, 0, 0, QByteArray(), &response, errorString)) {
        return false;
    }
    if (response.opcode != MAVFTPProtocol::OpAck) {
        *errorString = QStringLiteral("terminate failed: %1")
                .arg(MAVFTPProtocol::errorString(MAVFTPProtocol::responseErrorCode(response)));
        return false;
    }
    return true;
}

bool SitlFtpClient::sendMavlinkMessage(const mavlink_message_t& message, QString* errorString)
{
    uint8_t bytes[MAVLINK_MAX_PACKET_LEN];
    const uint16_t length = mavlink_msg_to_send_buffer(bytes, &message);
    const qint64 sent = _socket.writeDatagram(reinterpret_cast<const char*>(bytes), length, _vehicleAddress, _vehiclePort);
    if (sent != length) {
        *errorString = QStringLiteral("failed to send MAVLink datagram: %1").arg(_socket.errorString());
        return false;
    }
    return true;
}

bool SitlFtpClient::parseIncomingDatagrams(MAVFTPProtocol::Packet* response, bool* gotResponse, QString* errorString)
{
    *gotResponse = false;
    while (_socket.hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize(static_cast<int>(_socket.pendingDatagramSize()));
        QHostAddress sender;
        quint16 senderPort = 0;
        _socket.readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);

        for (int i = 0; i < datagram.size(); i++) {
            mavlink_message_t message;
            if (!mavlink_parse_char(MAVLINK_COMM_0, static_cast<uint8_t>(datagram.at(i)), &message, &_mavlinkStatus)) {
                continue;
            }
            if (message.msgid != MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL || !isResponseForUs(message)) {
                continue;
            }

            mavlink_file_transfer_protocol_t fileTransfer;
            mavlink_msg_file_transfer_protocol_decode(&message, &fileTransfer);
            if (!MAVFTPProtocol::decodePayload(fileTransfer.payload, MAVFTPProtocol::PayloadLength, response, errorString)) {
                return false;
            }
            *gotResponse = true;
            return true;
        }
    }
    return true;
}

bool SitlFtpClient::isResponseForUs(const mavlink_message_t& message) const
{
    if (message.sysid != _targetSystem || message.compid != _targetComponent) {
        return false;
    }

    mavlink_file_transfer_protocol_t fileTransfer;
    mavlink_msg_file_transfer_protocol_decode(&message, &fileTransfer);
    return (fileTransfer.target_system == 0 || fileTransfer.target_system == kGroundStationSystemId) &&
            (fileTransfer.target_component == 0 || fileTransfer.target_component == kGroundStationComponentId);
}

bool findParameter(const QList<MAVFTPFileFormats::ParameterValue>& parameters, const QString& name,
                   MAVFTPFileFormats::ParameterValue* parameter)
{
    foreach (const MAVFTPFileFormats::ParameterValue& candidate, parameters) {
        if (candidate.name == name) {
            *parameter = candidate;
            return true;
        }
    }
    return false;
}

bool verifyParameterValue(SitlFtpClient* client, const QString& name, double expectedValue, QString* errorString)
{
    QByteArray paramData;
    bool sawMultipleChunks = false;
    bool sawEofNack = false;
    if (!client->downloadFile(MAVFTPFileFormats::parameterDownloadPath(), &paramData, &sawMultipleChunks, &sawEofNack, errorString)) {
        return false;
    }

    QList<MAVFTPFileFormats::ParameterValue> parameters;
    if (!MAVFTPFileFormats::parseParameterFile(paramData, &parameters, errorString)) {
        return false;
    }

    MAVFTPFileFormats::ParameterValue parameter;
    if (!findParameter(parameters, name, &parameter)) {
        *errorString = QStringLiteral("parameter %1 not found after upload").arg(name);
        return false;
    }

    const double actualValue = variantToDouble(parameter.value);
    if (!nearlyEqual(actualValue, expectedValue)) {
        *errorString = QStringLiteral("parameter %1 value %2 did not match expected %3")
                .arg(name)
                .arg(actualValue)
                .arg(expectedValue);
        return false;
    }
    return true;
}

bool chooseSafeParameter(const QList<MAVFTPFileFormats::ParameterValue>& parameters,
                         MAVFTPFileFormats::ParameterValue* selected,
                         double* replacementValue,
                         QString* errorString)
{
    const QList<QPair<QString, QPair<double, double> > > candidates = {
        qMakePair(QStringLiteral("LOG_DISARMED"), qMakePair(0.0, 1.0)),
        qMakePair(QStringLiteral("WP_YAW_BEHAVIOR"), qMakePair(0.0, 1.0)),
        qMakePair(QStringLiteral("RTL_ALT"), qMakePair(1500.0, 1600.0)),
        qMakePair(QStringLiteral("WPNAV_SPEED"), qMakePair(500.0, 600.0)),
        qMakePair(QStringLiteral("PILOT_SPEED_UP"), qMakePair(250.0, 300.0))
    };

    foreach (const QPair<QString, QPair<double, double> >& candidate, candidates) {
        MAVFTPFileFormats::ParameterValue parameter;
        if (!findParameter(parameters, candidate.first, &parameter)) {
            continue;
        }
        if (parameter.packedType < 1 || parameter.packedType > 4) {
            continue;
        }
        const double currentValue = variantToDouble(parameter.value);
        *selected = parameter;
        *replacementValue = nearlyEqual(currentValue, candidate.second.first) ? candidate.second.second : candidate.second.first;
        logInfo(QStringLiteral("Selected parameter %1: current=%2 replacement=%3")
                .arg(parameter.name)
                .arg(currentValue)
                .arg(*replacementValue));
        return true;
    }

    *errorString = QStringLiteral("could not find a safe writable parameter candidate");
    return false;
}

bool runParameterRoundTrip(SitlFtpClient* client, QString* errorString)
{
    QByteArray paramData;
    bool sawMultipleChunks = false;
    bool sawEofNack = false;
    if (!client->downloadFile(MAVFTPFileFormats::parameterDownloadPath(), &paramData, &sawMultipleChunks, &sawEofNack, errorString)) {
        return false;
    }
    if (!sawMultipleChunks || !sawEofNack) {
        *errorString = QStringLiteral("parameter download did not prove chunking and EOF handling");
        return false;
    }

    QList<MAVFTPFileFormats::ParameterValue> parameters;
    if (!MAVFTPFileFormats::parseParameterFile(paramData, &parameters, errorString)) {
        return false;
    }
    if (parameters.isEmpty()) {
        *errorString = QStringLiteral("parameter download parsed zero parameters");
        return false;
    }
    logInfo(QStringLiteral("Parsed %1 parameters from SITL packed parameter file").arg(parameters.count()));

    MAVFTPFileFormats::ParameterValue selected;
    double replacementValue = 0.0;
    if (!chooseSafeParameter(parameters, &selected, &replacementValue, errorString)) {
        return false;
    }
    const double originalValue = variantToDouble(selected.value);

    QMap<QString, QVariant> upload;
    upload.insert(selected.name, uploadVariantFor(selected, replacementValue));
    QByteArray uploadData;
    if (!MAVFTPFileFormats::encodeParameterUploadFile(upload, &uploadData, errorString)) {
        return false;
    }

    bool uploadChunked = false;
    if (!client->uploadFile(MAVFTPFileFormats::parameterUploadPath(), uploadData, &uploadChunked, errorString)) {
        return false;
    }
    QThread::msleep(500);
    if (!verifyParameterValue(client, selected.name, replacementValue, errorString)) {
        return false;
    }

    upload.clear();
    upload.insert(selected.name, uploadVariantFor(selected, originalValue));
    uploadData.clear();
    if (!MAVFTPFileFormats::encodeParameterUploadFile(upload, &uploadData, errorString)) {
        return false;
    }
    if (!client->uploadFile(MAVFTPFileFormats::parameterUploadPath(), uploadData, &uploadChunked, errorString)) {
        return false;
    }
    QThread::msleep(500);
    if (!verifyParameterValue(client, selected.name, originalValue, errorString)) {
        return false;
    }

    logInfo(QStringLiteral("Parameter MAVFTP download/upload/reset verified for %1").arg(selected.name));
    return true;
}

mavlink_mission_item_int_t makeMissionItem(uint16_t seq, uint16_t command, int32_t lat, int32_t lon, float alt)
{
    mavlink_mission_item_int_t item;
    memset(&item, 0, sizeof(item));
    item.seq = seq;
    item.command = command;
    item.target_system = 1;
    item.target_component = 1;
    item.frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    item.current = seq == 0 ? 1 : 0;
    item.autocontinue = 1;
    item.mission_type = MAV_MISSION_TYPE_MISSION;
    item.x = lat;
    item.y = lon;
    item.z = alt;
    return item;
}

bool verifyMission(const QList<mavlink_mission_item_int_t>& items, QString* errorString)
{
    if (items.count() != 2) {
        *errorString = QStringLiteral("expected 2 mission items, got %1").arg(items.count());
        return false;
    }
    if (items.at(0).command != MAV_CMD_NAV_TAKEOFF || items.at(1).command != MAV_CMD_NAV_WAYPOINT) {
        *errorString = QStringLiteral("downloaded mission commands did not match uploaded mission");
        return false;
    }
    if (items.at(0).x != 473977420 || items.at(1).y != 85459400) {
        *errorString = QStringLiteral("downloaded mission coordinates did not match uploaded mission");
        return false;
    }
    if (std::fabs(items.at(0).z - 20.0f) > 0.001f || std::fabs(items.at(1).z - 25.0f) > 0.001f) {
        *errorString = QStringLiteral("downloaded mission altitudes did not match uploaded mission");
        return false;
    }
    return true;
}

bool runMissionRoundTrip(SitlFtpClient* client, QString* errorString)
{
    QList<mavlink_mission_item_int_t> mission;
    mission.append(makeMissionItem(0, MAV_CMD_NAV_TAKEOFF, 473977420, 85459400, 20.0f));
    mission.append(makeMissionItem(1, MAV_CMD_NAV_WAYPOINT, 473978420, 85459400, 25.0f));

    const QByteArray uploadData = MAVFTPFileFormats::encodeMissionFile(mission);
    bool uploadChunked = false;
    if (!client->uploadFile(MAVFTPFileFormats::missionPath(), uploadData, &uploadChunked, errorString)) {
        return false;
    }

    QByteArray downloadData;
    bool sawMultipleChunks = false;
    bool sawEofNack = false;
    if (!client->downloadFile(MAVFTPFileFormats::missionPath(), &downloadData, &sawMultipleChunks, &sawEofNack, errorString)) {
        return false;
    }
    if (!sawEofNack) {
        *errorString = QStringLiteral("mission download did not prove EOF handling");
        return false;
    }

    QList<mavlink_mission_item_int_t> downloadedMission;
    if (!MAVFTPFileFormats::parseMissionFile(downloadData, &downloadedMission, errorString)) {
        return false;
    }
    if (!verifyMission(downloadedMission, errorString)) {
        return false;
    }

    const QByteArray emptyMission = MAVFTPFileFormats::encodeMissionFile(QList<mavlink_mission_item_int_t>());
    if (!client->uploadFile(MAVFTPFileFormats::missionPath(), emptyMission, &uploadChunked, errorString)) {
        return false;
    }

    logInfo(QStringLiteral("Mission MAVFTP upload/download/clear verified"));
    return true;
}

} // namespace

int main(int argc, char* argv[])
{
    QCoreApplication app(argc, argv);

    bool ok = false;
    const int port = qEnvironmentVariableIntValue("MAVFTP_SITL_UDP_PORT", &ok);
    const int udpPort = ok ? port : kDefaultUdpPort;

    QString error;
    SitlFtpClient client;
    if (!client.connectToVehicle(udpPort, &error)) {
        logError(error);
        return 1;
    }
    if (!client.expectMissingFileNack(&error)) {
        logError(error);
        return 1;
    }
    if (!runParameterRoundTrip(&client, &error)) {
        logError(error);
        return 1;
    }
    if (!runMissionRoundTrip(&client, &error)) {
        logError(error);
        return 1;
    }

    logInfo(QStringLiteral("MAVFTP SITL integration test passed"));
    return 0;
}
