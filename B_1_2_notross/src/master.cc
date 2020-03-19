//
// Created by suraj on 3/19/20.
//

#include<string>
#include <vector>
#include <iostream>

#include "master.h"

using namespace std;

Master::Master() {
    cout << "<Master>\t" << "Initialised" << endl;
}

void Master::subscribe(Subscriber *subscriber) {
    this->topic_subscribers[subscriber->topic].push_back(subscriber);
    cout << "<Master>\t" << "New subscriber (" << subscriber->name << ") for " << subscriber->topic << endl;
}

void Master::publish(const string &topic, const string &message) {
    for (Subscriber *s : this->topic_subscribers[topic]) {
        s->recv_message(message);
    }
    cout << "<Master>\t" << "Published '" << message << "' to " << topic << endl;
}

Master *Master::master_instance = nullptr;

Master *Master::master() {
    if (master_instance == nullptr) {
        master_instance = new Master();
    }
    return master_instance;
}
