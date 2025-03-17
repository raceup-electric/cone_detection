# Theoretically-Efficient and Practical Parallel DBSCAN

This implementation of DBSCAN is based on the original work found in the [dbscan-python](https://github.com/wangyiqiu/dbscan-python) repository.

## Modifications
The `include/dbscan/` directory was copied as-is from the original repository, with only one modification:
- In `parallel.h`, the following changes were made to address multi-threading issues, which resulted in single-threaded execution:
  - Commented out `#define HOMEGROWN`
  - Added `#include <atomic>`

For more details, refer to the original repository.
