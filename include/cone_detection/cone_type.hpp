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
  const float SMALL_CONE_MIN_HEIGHT = 0.1;
  const float SMALL_CONE_MAX_HEIGHT = 0.3;
  const float SMALL_CONE_BASE_RADIUS = 0.20;

  const float BIG_CONE_MIN_HEIGHT = 0.3;
  const float BIG_CONE_MAX_HEIGHT = 0.6;
  const float BIG_CONE_BASE_RADIUS = 0.26;
}  

#endif  
