#include "gtest/gtest.h"
#include <ros/ros.h>
#include "../include/clustering.h"


TEST(ParseTest, Parse1Cluster) { 
    int cluster;
    int command;
    std::string msg = "C123#987";
    parseCommand(msg, &cluster, &command);
    ASSERT_EQ(cluster, 123); 
}
TEST(ParseTest, Parse1Command) { 
    int cluster;
    int command;
    std::string msg = "C123#987";
    parseCommand(msg, &cluster, &command);
    ASSERT_EQ(command, 987); 
}

TEST(ParseTest, Parse2Cluster) { 
    int cluster;
    int command;
    std::string msg = "C1#9";
    parseCommand(msg, &cluster, &command);
    ASSERT_EQ(cluster, 1); 
}
TEST(ParseTest, Parse2Command) { 
    int cluster;
    int command;
    std::string msg = "C1#9";
    parseCommand(msg, &cluster, &command);
    ASSERT_EQ(command, 9); 
}

TEST(ParseTest, Parse3Cluster) { 
    int cluster;
    int command;
    std::string msg = "C15#9";
    parseCommand(msg, &cluster, &command);
    ASSERT_EQ(cluster, 15); 
}
TEST(ParseTest, Parse3Command) { 
    int cluster;
    int command;
    std::string msg = "C15#9";
    parseCommand(msg, &cluster, &command);
    ASSERT_EQ(command, 9); 
}

TEST(ParseTest, Parse4Cluster) { 
    int cluster;
    int command;
    std::string msg = "C1#90";
    parseCommand(msg, &cluster, &command);
    ASSERT_EQ(cluster, 1); 
}
TEST(ParseTest, Parse4Command) { 
    int cluster;
    int command;
    std::string msg = "C1#90";
    parseCommand(msg, &cluster, &command);
    ASSERT_EQ(command, 90); 
}

TEST(ParseTest, Parse5Cluster) { 
    int cluster;
    int command;
    std::string msg = "C01#02";
    parseCommand(msg, &cluster, &command);
    ASSERT_EQ(cluster, 1); 
}
TEST(ParseTest, Parse5Command) { 
    int cluster;
    int command;
    std::string msg = "C01#02";
    parseCommand(msg, &cluster, &command);
    ASSERT_EQ(command, 2); 
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}