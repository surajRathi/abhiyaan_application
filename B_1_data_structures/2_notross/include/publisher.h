//
// Created by suraj on 3/19/20.
//

#ifndef B_1_2_NOTROSS_PUBLISHER_H
#define B_1_2_NOTROSS_PUBLISHER_H

#include <string>

class Publisher {
    std::string name;
    std::string topic;
public:
    Publisher(std::string name, std::string topic);

    void publish(const std::string &message);
};


#endif //B_1_2_NOTROSS_PUBLISHER_H
