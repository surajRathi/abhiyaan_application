//
// Created by suraj on 3/19/20.
//

#ifndef B_1_2_NOTROSS_MASTER_H
#define B_1_2_NOTROSS_MASTER_H

#include <string>
#include <map>
#include <vector>

#include "subscriber.h"

class Master {
    static Master *master_instance;

    Master();

    //std::map<std::string, std::vector<Subscriber *> > topic_publishers;
    std::map<std::string, std::vector<Subscriber *> > topic_subscribers;

public:
    static Master *master();

    void subscribe(Subscriber *subscriber);

    void publish(const std::string &topic, const std::string &message);
};

#endif //B_1_2_NOTROSS_MASTER_H
