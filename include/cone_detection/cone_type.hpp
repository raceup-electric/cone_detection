#ifndef CONE_TYPE_HPP
#define CONE_TYPE_HPP

namespace cone_detection {

  enum class ConeType {
    BIG_ORANGE,
    BLUE,
    YELLOW,
    ORANGE,
    UNKNOWN
  };

    // Cone size constraints
  const float SMALL_CONE_HEIGHT = 0.325;
  const float BIG_CONE_HEIGHT = 0.505;
  const float DIM_THRESHOLD = 0.1;
  
  const float SMALL_CONE_MIN_HEIGHT = SMALL_CONE_HEIGHT - DIM_THRESHOLD;
  const float SMALL_CONE_MAX_HEIGHT = SMALL_CONE_HEIGHT + DIM_THRESHOLD;
  const float SMALL_CONE_BASE_RADIUS = 0.114;



  const float BIG_CONE_MIN_HEIGHT = BIG_CONE_HEIGHT - DIM_THRESHOLD;
  const float BIG_CONE_MAX_HEIGHT = BIG_CONE_HEIGHT + DIM_THRESHOLD;
  const float BIG_CONE_BASE_RADIUS = 0.143; 
}  

#endif  
