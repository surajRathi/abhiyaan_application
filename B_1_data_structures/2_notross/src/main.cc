#include "master.h"

#include "subscriber.h"
#include "publisher.h"

using namespace std;

int main() {
    Subscriber s1("s1", "a_topic"), s2("s2", "a_topic");

    Master::master()->publish("a_topic", "Hello, a_topic.");
    Master::master()->publish("random", "123456789");

    Publisher p1("p1", "a_topic");

    p1.publish("Published from a Publisher.");

    return 0;
}
