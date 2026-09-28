#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <stdexcept>

#include "pty.h"

int pty_open_raw(const std::string &path)
{
	const int fd = ::open(path.c_str(), O_RDWR | O_NOCTTY);
	if (fd < 0) {
		throw std::runtime_error("cannot open " + path + ": " + std::strerror(errno));
	}
	termios t{};
	if (::tcgetattr(fd, &t) != 0) {
		const int e = errno;
		::close(fd);
		throw std::runtime_error("tcgetattr " + path + ": " + std::strerror(e));
	}
	::cfmakeraw(&t);
	if (::tcsetattr(fd, TCSANOW, &t) != 0) {
		const int e = errno;
		::close(fd);
		throw std::runtime_error("tcsetattr " + path + ": " + std::strerror(e));
	}
	return fd;
}
