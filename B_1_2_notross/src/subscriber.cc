//
// Created by suraj on 3/19/20.
//

#include "subscriber.h"

#include <iostream>
#include <string>

#include "master.h"

using namespace std;

Subscriber::Subscriber(string name, string topic) : name(move(name)), topic(move(topic)) {
    Master::master()->subscribe(this);
    cout << "<" << this->name << ">\tCreated for " << this->topic << endl;
}

void Subscriber::recv_message(const string &message) {
    this->messages.push_back(message);
    cout << "<" << this->name << ">\tReceived message: " << message << endl;
}

vector<std::string> Subscriber::get_messages() {
    return vector<std::string>(this->messages);
}
