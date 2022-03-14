# Gaia MQTT example application
A Gaia application to demonstrate communicating via MQTT.

# MQTT example development setup
## Prerequisites
You'll need to

* Install Gaia
* Install aws-iot-device-sdk-cpp

### Installing Gaia
Follow instructions [here](https://gaia-platform.github.io/gaia-platform-docs.io/articles/getting-started-with-gaia.html) to install Gaia.

### Installing aws-iot-device-sdk-cpp
From inside the root sandbox directory clone the [aws-iot-device-sdk-cpp](https://github.com/aws/aws-iot-device-sdk-cpp-v2) and build using clang. May also work with gcc but so far only tested with clang.
```bash
export CC=/usr/bin/clang-10
export CPP=/usr/bin/clang-cpp-10
export CXX=/usr/bin/clang++-10
export LDFLAGS=-fuse-ld=lld-10
cd {mqtt_example_root_directory}
git clone --recursive https://github.com/aws/aws-iot-device-sdk-cpp-v2.git
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
* Create a new folder named `certs` in your mqtt directory and copy the downloaded files there.
* Rename the certificate and private key respectively to: `certificate.pem.crt` and `private.pem.key`
* Leave the Amazon root certificate name unchanged, namely: `AmazonRootCA1.pem`

## Build MQTT example
Create build directory within mqtt example directory, run cmake and then make.
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
