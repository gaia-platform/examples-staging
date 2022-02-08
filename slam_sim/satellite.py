#!/usr/bin/env python3
"""
Web endpoint that serves SLAM info, including map, robot location, and
robot sensory input, and more should tne need arise. All data transfer 
is via json.

Note that the simulator does not perform collision detection. Objects in
the world can be moved through (imagine them as very thin fog walls).

World dimensions are encoded in viewer's .js and .html files.

Input data format
-----------------

Endpoint: /position/
Action: Resets position of robot.
JSON:
{
  x: 123.45,
  y: 234.56,
  heading: 225.0
}
Simulator recieves above packet with additional field:
  type: "position"


Endpoint: /motion/
Action: Report robot motion.
JSON:
{
  distance: 5.0,
  turn_angle: 0.5
}
Simulator recieves above packet with additional field:
  type: "motion"


Endpoint: /get-map/
Action: Requests map of world (collection of line segments)
JSON:
  None
Simulator recieves json packet with field:
  type: "get-map"


Expansion considerations
- Input JSON can include objects in the world that move (e.g., other robots).
- Map can be sent in from external process, perhaps to /set-map/.


Output data format
------------------

Endpoint: /get-map/
Data: Returns line segments that outline objects in world.
JSON:
{
  description: "map description",
  status: "ready",
  width: 1000,
  height: 600,
  world: [
    {
      name: "object-1" 
      vertices: [
        { x: 1.0, y: 1.0 },
        { x: 2.0, y: 1.0 },
        { x: 2.0, y: 2.0 },
        { x: 1.0, y: 2.0 },
        { x: 1.0, y: 1.0 },
      ]
    },
    {
      name: "object-2"
      vertices: [
        ...
      ]
    }
  ]
}

Endpoint: /motion/ or /position/
Data:
{
  position: {     -- actual position
    x: 123.45,
    y: 234.56,
    heading_deg: 225.0
  }
  estimation: {   -- position robot thinks it's in
    x: 123.45,
    y: 234.56,
    heading_deg: 225.0
  }
  senses: {       -- environmental mesurements
    motion: {     -- net motion from IMU and/or wheel sensors
      dx: 2.34,   --   this will be 0.0 when position is being set
      dy: 4.56,
      dhead: -3.3
    }
    lidar: [      -- data points returned from lidar, abs & rel
      {
        x: 125.0,
        y: 235.0,
        bearing: 45.0,
        range: 5.6
      },
      {
        ...
      }
    ]
  }
}

Endpoint acts only as a relay of data from simulation core.

"""
from flask import Flask, jsonify, make_response, request
from flask_cors import CORS
from datetime import datetime
import requests
import socket
import json

import dummy

SIM_HOST= "localhost"
SIM_PORT = 4500

# This process is the 'satellite' as it relays data.
SATELLITE_PORT = 4501


PACKET_TYPE_STR = "type"
MAP_PACKET = "get-map"
MOTION_PACKET = "motion"
POSITION_PACKET = "position"


# Flask. CORS is needed to avoid authentication problems
app = Flask(__name__)
CORS(app)


########################################################################


@app.errorhandler(404)
def not_found(error):
    msg = {}
    msg["content"] = "Page not found"
    msg["code"] = 404
    return msg


# Returns json packet returned from simulator, or error packet.
def send_packet(content):
    response = ""
    try:
        content = json.dumps(dict(content))
        ########################################
        # Establish connection to simulator to relay data.
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((SIM_HOST, SIM_PORT))
        print("Connected to simulator")
        ########################################
        # Send packet. Header first, then content.
        # Header is 16 character string storing the length message in
        #   bytes.
        size = len(content);
        header = "%16d" % size
        s.send(header.encode('utf-8'))
        # Send body.
        tot_sent = 0
        while tot_sent < len(content):
            print("Sending", content[tot_sent:])
            sent = s.send(content[tot_sent:].encode('utf-8'))
            print("Sent %d of %d bytes" % (sent, len(content)))
            if sent == 0:
                raise RuntimeError("Socket connection broken")
            tot_sent += sent
        #######################
        # TODO receive response
        header = s.recv(16);
        size = int(header);
        print("Receiving %d bytes" % size)
        tot_received = 0
        bytes_remaining = size
        while tot_received < size:
            packet = s.recv(bytes_remaining)
            response += packet.decode('utf-8')
            bytes_received = len(packet)
            tot_received += bytes_received
            bytes_remaining -= bytes_received
    except Exception as e:
        err = "Failed to send data to simulator: %s\n" % str(e)
        print(err)
        response = {}
        response["error"] = err;
    return response

@app.route('/get-map/', methods=['GET'])
def get_map():
    message = {}
    message[PACKET_TYPE_STR] = MAP_PACKET
    response = send_packet(message)
    return make_response(response)


@app.route('/position/', methods=['POST'])
def set_position():
    message = dict(request.get_json())
    message[PACKET_TYPE_STR] = POSITION_PACKET
    response = send_packet(message)
    return make_response(response)


@app.route('/motion/', methods=['POST'])
def robot_motion():
    message = dict(request.get_json())
    message[PACKET_TYPE_STR] = MOTION_PACKET
    response = send_packet(message)
    return make_response(response)


@app.route('/get-map/dummy/', methods=['GET'])
def get_map_dummy():
    return make_response(dummy.world_map)


@app.route('/position/dummy/', methods=['GET'])
def get_position_dummy():
    message = dummy.position
    print(message)
    message[PACKET_TYPE_STR] = POSITION_PACKET
    response = send_packet(message)
    return make_response(dummy.sensor_data)


@app.route('/motion/dummy/', methods=['GET'])
def robot_motion_dummy():
    message = dummy.motion
    message[PACKET_TYPE_STR] = MOTION_PACKET
    response = send_packet(message)
    return make_response(dummy.sensor_data)


@app.route('/healthcheck/', methods=['GET'])
def is_alive():
    return make_response({ "status": "alive" })


########################################################################


if __name__ == '__main__':
    app.run('0.0.0.0', SATELLITE_PORT, debug=True)

