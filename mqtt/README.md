# Gaia MQTT example application
A Gaia application to demonstrate communicating via MQTT.

# MQTT example development setup
## Prerequisites
You'll need to:

* Install Gaia.
* Install [aws-iot-device-sdk-cpp](https://github.com/aws/aws-iot-device-sdk-cpp-v2).

### Installing Gaia
Follow instructions [here](https://gaia-platform.github.io/gaia-platform-docs.io/articles/getting-started-with-gaia.html) to install Gaia.

### Installing aws-iot-device-sdk-cpp
Clone the [aws-iot-device-sdk-cpp](https://github.com/aws/aws-iot-device-sdk-cpp-v2) and build.

```bash
git clone --recursive --branch v1.14.2 https://github.com/aws/aws-iot-device-sdk-cpp-v2.git
cd aws-iot-device-sdk-cpp-v2
mkdir build
cd build
cmake ..
make
sudo make install
```

## Install AWS IoT certificates
* [Create a thing](https://aws.amazon.com/iot/), a certificate for your thing, and a policy to attach it to.
* Download the new certificate, private key, and Amazon root CA1 certificate.
* Create a new folder named `certs` in your `mqtt` directory and copy the downloaded files there.
* Rename the certificate and private key respectively to: `certificate.pem.crt` and `private.pem.key`
* Leave the Amazon root certificate name unchanged, namely: `AmazonRootCA1.pem`

## Build MQTT example
Follow these steps:
```bash
cd {mqtt_example_root_directory}
mkdir build
cd build
cmake ..
make
```

## Run the example
```bash
./mqtt
```

## Send test MQTT messages

The AWS Console provides a convenient MQTT test client.

You can find the instructions here: https://docs.aws.amazon.com/iot/latest/developerguide/view-mqtt-messages.html

You can publish to the topic `my_thing_name/`.
