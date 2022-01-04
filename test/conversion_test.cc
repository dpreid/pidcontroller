//conversion_unittest.cpp

#include <gtest/gtest.h>

TEST(Conversion, Millis) {

  // if the conversion fails like issue #4 then
  // t0 will equal t1 for some big numbers of i
  // if this test fails, make other tests to
  // narrow down what value of i this conversion fails for
  for (long int i =1; i < 2147483647; i = i *2)
	{
	  float t0 = (float) i;
	  float t1 = (float) i+1;
	  EXPECT_TRUE(t1 > t0);
		
	}
 
}

TEST(Conversion, FourNinesMillis) {

  unsigned long i = 	9999;  
  float t0 = (float) i;
  float t1 = (float) i+1;
  EXPECT_TRUE(t1 > t0);
  
}

TEST(Conversion, SixNinesMillis) {

  unsigned long i = 	999999;  
  float t0 = (float) i;
  float t1 = (float) i+1;
  EXPECT_TRUE(t1 > t0);
  
}
TEST(Conversion, SevenNinesMillis) {

  unsigned long i = 	9999999;  
  float t0 = (float) i;
  float t1 = (float) i+1;
  EXPECT_TRUE(t1 > t0);
  
}

TEST(Conversion, TwoPower23Millis) {

  unsigned long i = 	8388608;
  float t0 = (float) i;
  float t1 = (float) i+1;
  EXPECT_TRUE(t1 > t0);
  
}

TEST(Conversion, JustUnderTwoPower24Millis) {

  unsigned long i = 	16777210;  
  float t0 = (float) i;
  float t1 = (float) i+1;
  EXPECT_TRUE(t1 > t0);
  
}

TEST(Conversion, TwoPower24MillisDelta4) {

  unsigned long i = 	16777216;  
  float t0 = (float) i;
  float t1 = (float) i+4;
  EXPECT_TRUE(t1 > t0);
  
}

TEST(Conversion, JustUnderTwoPower26MillisDelta4) {

  unsigned long i = 67108859;	
  float t0 = (float) i;
  float t1 = (float) i+4;
  EXPECT_TRUE(t1 > t0);
  
}

TEST(Conversion, TwoPower26MillisDelta4) {

  unsigned long i = 67108864;	
  float t0 = (float) i;
  float t1 = (float) i+4;
  EXPECT_TRUE(t1 > t0);
  
}


TEST(Conversion, TwoPower24Millis) {

  unsigned long i = 	16777216;  
  float t0 = (float) i;
  float t1 = (float) i+1;
  EXPECT_TRUE(t1 > t0);
  
}

TEST(Conversion, TwoPower25Millis) {

  unsigned long i = 	33554432;  
  float t0 = (float) i;
  float t1 = (float) i+1;
  EXPECT_TRUE(t1 > t0);
  
}

TEST(Conversion, TwoPower26Millis) {

  unsigned long i = 	67108864;  
  float t0 = (float) i;
  float t1 = (float) i+1;
  EXPECT_TRUE(t1 > t0);
  
}

TEST(Conversion, EightNinesMillis) {

  unsigned long i = 	99999999;  
  float t0 = (float) i;
  float t1 = (float) i+1;
  EXPECT_TRUE(t1 > t0);
  
}

TEST(Conversion, NineNinesMillis) {

  unsigned long i = 	999999999;  
  float t0 = (float) i;
  float t1 = (float) i+1;
  EXPECT_TRUE(t1 > t0);
  
}
TEST(Conversion, MaxMillis) {

  long int i = 	2147483646;  
  float t0 = (float) i;
  float t1 = (float) i+1;
  EXPECT_TRUE(t1 > t0);
  
}
