#ifndef APP_COMMON_H
#define APP_COMMON_H

// Function to check for and handle status queries
// Apps should call this periodically in their main loops
void check_for_status_query(void);

#endif // APP_COMMON_H