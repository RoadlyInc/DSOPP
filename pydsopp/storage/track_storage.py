import sys, os

sys.path.insert(0, os.path.abspath(os.path.join("${STORAGE_PATH}", "proto")))

import agent_settings_pb2 as agent_settings
import ecef_poses_pb2 as ecef_poses
import frame_pb2 as frame
import gnss_track_pb2 as gnss_track
import connection_pb2 as connection
import sanity_check_results_pb2 as sanity_check_results

LONG_UNSIGNED_SIZE = 8
UNSIGNED_SIZE = 4


## Saves one proto message
#  @param file opened file for writing
#  @param proto_message protobuf container
def saveProto(file, proto_message):

    message_size = proto_message.ByteSize()
    message_size_bytes = message_size.to_bytes(UNSIGNED_SIZE,
                                               byteorder='little')
    file.write(message_size_bytes)
    file.write(proto_message.SerializeToString())


## Reads one proto message
#  @param file opened file for reading
#  @param proto_message protobuf container
#  @return proto message
def readProto(file, proto_message):

    message_size_bytes = file.read(UNSIGNED_SIZE)
    message_size = int.from_bytes(message_size_bytes, byteorder='little')
    proto_message_bytes = file.read(message_size)
    proto_message.ParseFromString(proto_message_bytes)
    return proto_message


## Class for storing data
class TrackStorage:

    ## Saves data in file
    #  @param self object pointer
    #  @param filename filename string
    #  @return
    def save(self, filename):

        file = open(filename, 'wb')
        number_of_frames = len(self.frames)
        number_of_frames_bytes = number_of_frames.to_bytes(LONG_UNSIGNED_SIZE,
                                                           byteorder='little')
        file.write(number_of_frames_bytes)
        for frame in self.frames:
            saveProto(file, frame)

        saveProto(file, self.connections)
        saveProto(file, self.gnss_track)
        saveProto(file, self.ecef_poses)
        saveProto(file, self.sanity_check_results)
        saveProto(file, self.agent_settings)

        file.close()

    ## Reads data from file
    #  @param self object pointer
    #  @param filename filename string
    #  @return
    def read(self, filename):

        file = open(filename, 'rb')

        number_of_frames_bytes = file.read(LONG_UNSIGNED_SIZE)
        number_of_frames = int.from_bytes(number_of_frames_bytes,
                                          byteorder='little')

        self.frames = [
            readProto(file, frame.Keyframe()) for i in range(number_of_frames)
        ]
        self.connections = readProto(file, connection.Connections())
        self.gnss_track = readProto(file, gnss_track.GnssTrack())
        self.ecef_poses = readProto(file, ecef_poses.ECEFPoses())
        self.sanity_check_results = readProto(
            file, sanity_check_results.SanityCheckResults())
        self.agent_settings = readProto(file, agent_settings.AgentSettings())

        file.close()

    ## @var frames
    #  protobuf containers for frames
    ## @var connections
    #  protobuf container for connections
    ## @var gnss_track
    #  protobuf container for gnss track
    ## @var ecef_poses
    #  protobuf container for ecef poses
    ## @var sanity_check_results
    #  protobuf container for sanity check results
    ## @var agent_settings
    #  protobuf container for agent settings
