//
// Created by suraj on 3/19/20.
//

#ifndef B_1_2_NOTROSS_SUBSCRIBER_H
#define B_1_2_NOTROSS_SUBSCRIBER_H

#include <string>
#include <vector>

class Subscriber {
    std::vector<std::string> messages;
public:
    const std::string name;
    const std::string topic;

    Subscriber(std::string name, std::string topic);

    void recv_message(const std::string &message);

    std::vector<std::string> get_messages();
};


#endif //B_1_2_NOTROSS_SUBSCRIBER_H
