
#include "state.h"
#include "al/al_console.h"

extern "C" AL_EXPORT int onload(void) {
	printf("onloaded from project!\n");
	return 0;
}

extern "C" AL_EXPORT int onunload(void) {
	printf("unloading from project!\n");
	return 0;
}