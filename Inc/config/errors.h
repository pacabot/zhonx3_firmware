/* errors.h */
#ifndef __ERRORS_H__
#define __ERRORS_H__

#ifndef MAKE_ERROR
# define MAKE_ERROR(_id_, _errno_) (((_id_) << 8) | (_errno_))
#endif

//#define SUCCESS                         0
#define ERR_NO_MORE_HANDLE_AVAILABLE    0x100001

#endif // __ERRORS_H__
