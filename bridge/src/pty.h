#ifndef BRIDGE_PTY_H_
#define BRIDGE_PTY_H_

#include <string>

/* Opens a PTY slave in raw mode; throws std::runtime_error on failure. */
int pty_open_raw(const std::string &path);

#endif /* BRIDGE_PTY_H_ */
