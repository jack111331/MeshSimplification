#include <pthread.h>
void ThreadCleanupFunc(void *p) { };

int main(int argc, char* argv[]) {
 
        void *p;
        pthread_cleanup_push(ThreadCleanupFunc, p);
        pthread_cleanup_pop(0);
return 0; }
