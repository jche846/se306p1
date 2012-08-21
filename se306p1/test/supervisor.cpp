#include <gtest/gtest.h>
#include <supervisor.h>

//Begin to implement tests in supervisor
using namespace se306p1;
using namespace Supervisor;
namespace {
class SupervisorTest : public testing::Test {
   
   public:
    
    SupervisorTest() {
      ros::NodeHandle nh("~");
      Supervisor super(nh);
    }

    virtual ~SupervisorTest() {
    }

    virtual void SetUp() {
    }

    virtual void TearDown() {
    }
  };

  /**
  * 
  */
  TEST_F(SupervisorTest, testSwitchBehaviour) {
    uint64_t id = 1;
    SwitchBehavior(id);
    
  }
  
int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
