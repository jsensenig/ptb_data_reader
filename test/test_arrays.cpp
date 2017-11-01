#include <iostream>
#include <cstdlib>
#include <cstdio>

int main() {

	unsigned int *orig = new unsigned int[4];
	for (unsigned int i = 0; i < 4; ++i) {
		orig[i] = i+1;
	}
	//unsigned int orig[] = {1,2,3,4};
	printf("Original %08X (%u %u %u %u)\n",(unsigned int)orig,orig[0],orig[1],orig[2],orig[3]);
	unsigned int ptr[4];
	for (unsigned int i = 0; i < 4; ++i) {
		ptr[i] = orig[3-i];
	}
	delete [] orig;
	printf("Original %08X (%u %u %u %u)\n",(unsigned int)orig,orig[0],orig[1],orig[2],orig[3]);
	printf("Copy %08X (%u %u %u %u)\n",(unsigned int)ptr,ptr[0],ptr[1],ptr[2],ptr[3]);
	return 0;
}
