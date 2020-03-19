//
// Created by suraj on 3/19/20.
//

#include "publisher.h"

#include <iostream>
#include <string>

#include "master.h"

using namespace std;

Publisher::Publisher(string name, string topic) : name(move(name)), topic(move(topic)) {
    cout << "<" << this->name << ">\t" << "New publisher for " << this->topic << endl;
}

void Publisher::publish(const string &message) {
    Master::master()->publish(this->topic, message);
    cout << "<" << this->name << ">\t" << "Publishing '" << message << "'" << endl;
}
