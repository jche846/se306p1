#include <gtest/gtest.h>
#include <se306p1/Do.h>
#include <se306p1/Go.h>
#include "../src/util/command.h"

// The fixture for testing class Command.
namespace {
class CommandTest : public testing::Test {
   public:
   
    se306p1::Do msg_do;
    se306p1::Go msg_go;
    
    CommandTest() {
      // You can do set-up work for each test here.
    }

    virtual ~CommandTest() {
      // You can do clean-up work that doesn't throw exceptions here.
    }

    virtual void SetUp() {
      msg_do.lv = 10;
      msg_do.av = 30;
      msg_do.enqueue = true;
      
      msg_go.x = 7;
      msg_go.y = 15;
      msg_go.theta = 33;
      msg_go.enqueue = false;
    }

    virtual void TearDown() {
    }
  };

  TEST_F(CommandTest, testDoConstruct) {
    se306p1::Command command(msg_do);
    ASSERT_EQ(msg_do.lv, command.lv);
    ASSERT_EQ(msg_do.av, command.av);
    ASSERT_EQ(msg_do.enqueue, command.enqueue);
  }

  TEST_F(CommandTest, testGoConstruct) {
    se306p1::Command command(msg_go);
    ASSERT_EQ(msg_go.x, command.x);
    ASSERT_EQ(msg_go.y, command.y);
    ASSERT_EQ(msg_go.theta, command.theta);
    ASSERT_EQ(msg_go.enqueue, command.enqueue);
  }

  TEST_F(CommandTest, testDoType) {
    se306p1::Command command(msg_do);
    ASSERT_TRUE(command.isDo);
  }

  TEST_F(CommandTest, testGoType) {
    se306p1::Command command(msg_go);
    ASSERT_FALSE(command.isDo);
  }
}//namespace

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
