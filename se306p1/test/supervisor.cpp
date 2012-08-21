#include <gtest/gtest.h>
#include <supervisor.h>

//Begin to implement tests in supervisor
using namespace se306p1;
namespace {
class SupervisorTest : public testing::Test {
   
   public:
    
    SupervisorTest() : nh("~"), rc(nh,1) {
      
    }

    virtual ~SupervisorTest() {
      
    }

    virtual void SetUp() {

    }

    virtual void TearDown() {
      
    }
  };

  /**
  * Tests that do_callback runs correctly when enqueue is true
  */
  TEST_F(SupervisorTest, test_do_callback_enqueue) {
    
  }
  
int main(int argc, char *argv[]) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
