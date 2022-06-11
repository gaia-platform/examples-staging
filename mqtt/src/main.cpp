////////////////////////////////////////////////////
// Copyright (c) Gaia Platform Authors
//
// Use of this source code is governed by the MIT
// license that can be found in the LICENSE.txt file
// or at https://opensource.org/licenses/MIT.
////////////////////////////////////////////////////

#include <iostream>

#include <aws/crt/Api.h>
#include <aws/iot/MqttClient.h>

#include "gaia/logger.hpp"
#include "gaia/rules/rules.hpp"
#include "gaia/system.hpp"

#include "gaia_mqtt.h"
#include "utils.hpp"

using namespace Aws::Crt;
using namespace std;

using namespace gaia::common;
using namespace gaia::db;
using namespace gaia::db::triggers;
using namespace gaia::direct_access;
using namespace gaia::mqtt;
using namespace gaia::rules;
using namespace gaia::mqtt::utils;

// When connecting to an AWS endpoint, use the ATS endpoint.
// Use this command to find it:
// $ aws iot describe-endpoint --endpoint-type iot:Data-ATS
#define ENDPOINT "replace_with_your_mqtt_endpoint"

std::shared_ptr<Aws::Crt::Mqtt::MqttConnection> g_connection;

auto g_on_publish_complete = [](Mqtt::MqttConnection&, uint16_t packet_id, int error_code) {
    if (packet_id)
    {
        gaia_log::app().debug("Operation on packet ID '{}' succeeded.", packet_id);
    }
    else
    {
        gaia_log::app().error("Operation failed with error '{}'.", aws_error_debug_str(error_code));
    }
};

void publish_message(const string& topic, const string& payload)
{
    if (g_connection)
    {
        gaia_log::app().info("Publishing: topic: '{}' payload: '{}'.", topic, trim_to_size(payload));
        ByteBuf payload_buf = ByteBufFromArray(reinterpret_cast<const uint8_t*>(payload.data()), payload.length());
        g_connection->Publish(topic.c_str(), AWS_MQTT_QOS_AT_LEAST_ONCE, false, payload_buf, g_on_publish_complete);
    }
}

void dump_db()
{
    // NYI
}

vector<string> split_topic(const string& topic)
{
    vector<string> result;
    size_t left = 0;
    size_t right = topic.find('/');
    while (right != string::npos)
    {
        result.push_back(topic.substr(left, right - left));
        left = right + 1;
        right = topic.find('/', left);
    }
    result.push_back(topic.substr(left));
    return result;
}

void on_message(
    Mqtt::MqttConnection&,
    const String& topic,
    const ByteBuf& payload,
    bool /*dup*/,
    Mqtt::QOS /*qos*/,
    bool /*retain*/)
{
    vector<string> topic_vector = split_topic(topic.c_str());
    string payload_str(reinterpret_cast<char*>(payload.buffer), payload.len);
    payload_str += '\0';
    gaia_log::app().info("Received topic: '{}' payload: '{}'.", topic.c_str(), trim_to_size(payload_str));

    begin_transaction();
    messages_t::insert_row(topic.c_str(), payload_str.c_str());
    commit_transaction();
}

int main()
{
    gaia::system::initialize();

    begin_transaction();
    dump_db();
    commit_transaction();

    ApiHandle api_handle;

    String endpoint(ENDPOINT);
    String certificate_path("../certs/certificate.pem.crt");
    String key_path("../certs/private.pem.key");
    String ca_file("../certs/AmazonRootCA1.pem");
    String client_id = get_uuid().c_str();

    Io::EventLoopGroup event_loop_group(1);
    if (!event_loop_group)
    {
        gaia_log::app().error(
            "Event Loop Group Creation failed with error '{}'.", ErrorDebugString(event_loop_group.LastError()));
        exit(-1);
    }

    Aws::Crt::Io::DefaultHostResolver default_host_resolver(event_loop_group, 1, 5);
    Io::ClientBootstrap bootstrap(event_loop_group, default_host_resolver);

    if (!bootstrap)
    {
        gaia_log::app().error("ClientBootstrap failed with error '{}'.", ErrorDebugString(bootstrap.LastError()));
        exit(-1);
    }

    Aws::Iot::MqttClientConnectionConfigBuilder builder;

    builder = Aws::Iot::MqttClientConnectionConfigBuilder(certificate_path.c_str(), key_path.c_str());
    builder.WithCertificateAuthority(ca_file.c_str());
    builder.WithEndpoint(endpoint);

    auto client_config = builder.Build();

    if (!client_config)
    {
        gaia_log::app().error("Client Configuration initialization failed with error '{}'.",
            ErrorDebugString(client_config.LastError()));
        exit(-1);
    }

    Aws::Iot::MqttClient mqtt_client(bootstrap);

    if (!mqtt_client)
    {
        gaia_log::app().error(
            "MQTT Client Creation failed with error '{}'.", ErrorDebugString(mqtt_client.LastError()));
        exit(-1);
    }

    g_connection = mqtt_client.NewConnection(client_config);

    if (!g_connection)
    {
        gaia_log::app().error(
            "MQTT Connection Creation failed with error '{}'.", ErrorDebugString(mqtt_client.LastError()));
        exit(-1);
    }

    std::promise<bool> connection_completed_promise;
    std::promise<void> connection_closed_promise;

    auto on_connection_completed = [&connection_completed_promise](Mqtt::MqttConnection&, int error_code, Mqtt::ReturnCode return_code, bool) {
        if (error_code)
        {
            gaia_log::app().error("Connection failed with error '{}'.", ErrorDebugString(error_code));
            connection_completed_promise.set_value(false);
        }
        else
        {
            if (return_code != AWS_MQTT_CONNECT_ACCEPTED)
            {
                gaia_log::app().error("Connection failed with MQTT return code '{}'.", static_cast<int>(return_code));
                connection_completed_promise.set_value(false);
            }
            else
            {
                gaia_log::app().info("Connection completed successfully.");
                gaia::system::initialize();
                connection_completed_promise.set_value(true);
            }
        }
    };

    auto on_interrupted = [&](Mqtt::MqttConnection&, int error) {
        gaia_log::app().error("Connection interrupted with error '{}'.", ErrorDebugString(error));
    };

    auto on_resumed = [&](Mqtt::MqttConnection&, Mqtt::ReturnCode, bool) {
        gaia_log::app().info("Connection resumed.");
    };

    auto on_disconnect = [&connection_closed_promise](Mqtt::MqttConnection&) {
        {
            gaia_log::app().info("Disconnect completed.");
            gaia::system::shutdown();
            connection_closed_promise.set_value();
        }
    };

    g_connection->OnConnectionCompleted = std::move(on_connection_completed);
    g_connection->OnDisconnect = std::move(on_disconnect);
    g_connection->OnConnectionInterrupted = std::move(on_interrupted);
    g_connection->OnConnectionResumed = std::move(on_resumed);

    gaia_log::app().info("Connecting...");
    if (!g_connection->Connect(client_id.c_str(), false, 1000))
    {
        gaia_log::app().error(
            "MQTT Connection failed with error '{}'.", ErrorDebugString(g_connection->LastError()));
        exit(-1);
    }

    if (connection_completed_promise.get_future().get())
    {
        std::promise<void> subscribe_finished_promise;
        auto on_sub_ack =
            [&subscribe_finished_promise](Mqtt::MqttConnection&, uint16_t packet_id, const String& topic, Mqtt::QOS qos, int error_code) {
                if (error_code)
                {
                    gaia_log::app().error("Subscribe failed with error '{}'.", aws_error_debug_str(error_code));
                    exit(-1);
                }
                else
                {
                    if (!packet_id || qos == AWS_MQTT_QOS_FAILURE)
                    {
                        gaia_log::app().error("Subscribe rejected by the broker.");
                        exit(-1);
                    }
                    else
                    {
                        gaia_log::app().info(
                            "Subscribe on topic '{}' on packet ID '{}' succeeded.", topic.c_str(), packet_id);
                    }
                }
                subscribe_finished_promise.set_value();
            };

        string topic = "my_thing_name/#";
        g_connection->Subscribe(topic.c_str(), AWS_MQTT_QOS_AT_LEAST_ONCE, on_message, on_sub_ack);
        subscribe_finished_promise.get_future().wait();

        String input;
        while (input != "x")
        {
            gaia_log::app().info("Press Enter to see database. Enter 'x' to exit this program.");
            std::getline(std::cin, input);
            if (input != "x")
            {
                begin_transaction();
                dump_db();
                commit_transaction();
            }
        }

        std::promise<void> unsubscribe_finished_promise;
        g_connection->Unsubscribe(
            "#",
            [&unsubscribe_finished_promise](Mqtt::MqttConnection&, uint16_t, int) { unsubscribe_finished_promise.set_value(); });
        unsubscribe_finished_promise.get_future().wait();
    }

    if (g_connection->Disconnect())
    {
        connection_closed_promise.get_future().wait();
    }

    gaia::system::shutdown();

    return 0;
}
