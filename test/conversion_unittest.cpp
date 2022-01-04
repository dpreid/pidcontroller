//conversion_unittest.cpp

#include <gtest/gtest.h>

TEST(conversion, millis) {

  // if the conversion fails like issue #4 then
  // t0 will equal t1 for big numbers of i
  for (long int i =1; i < 2147483647; i = i *2)
	{
	  float t0 = (float) i;
	  float t1 = (float) i+1;
	  EXPECT_TRUE(t1 > t0);
		
	}
  
}
