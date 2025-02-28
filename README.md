# Before you start the repository make sure you run through the changes below

Go to edge impulse -> deployment

1. Select the deployment to use C++ library
2. Build
3. Once build finished, download the latest build zip.
4. Copy all the directories in the zip into the root of this project. It contains our model and all the necessary edge impulse inference code.

# SDK config
Set the esp-idf SDK configuration (cmd+shift+p -> menuconfig).

* Enable SPIRAM. This ensure we have enough memory to work on image and edge impulse features
* Set Mode (QUAD/OCT) of SPI RAM chip in use SPIRAM_MODE to: `Octal Mode PSRAM`. So it has higher bandwidth for the imaging and ML task.
* Set Flash SPI speed to 80 mhz.
* Set `SPI RAM access method` to use `heap_caps_malloc(..., MALLOC_CAP_SPIRAM)`. This set `MALLOC_CAP_DEFAULT` to be `MALLOC_CAP_SPIRAM`. That is the default allocation to be SPI RAM.

All of these changes are just to ensure we have:

1. Enough memory
2. Enough memory bandwidth
3. Faster flash speed

So that there is no bottleneck for the edge impulse model, and image capturing process.

# Edge impulse C++ library bugs and traps
The function `ei_calloc` in `./edge-impulse-sdk/porting/espressif/ei_classifier_porting.cpp` has a bug. It is not allocating the consistent 16-byte alignment memory compare to `ei_malloc`, it might produce a memory error when invoking edge impulse `run_classifier` function:

```
Guru Meditation Error: Core  0 panic'ed (StoreProhibited). Exception was unhandled.
```

In our case, it is relate to the memory alignment. We need to make sure `ei_calloc` is allocate the same 16-byte alignment memory so it is consistent with the `ei_malloc`:

```c++
__attribute__((weak)) void *ei_calloc(size_t nitems, size_t size) {
    void *ptr = heap_caps_aligned_alloc(16, nitems * size, MALLOC_CAP_DEFAULT);
    if (ptr) {
        memset(ptr, 0, nitems * size);
    }
    return ptr;

}
```

Note that, make sure that SPIRAM is enabled in the esp-idf SDK config, so that `MALLOC_CAP_DEFAULT` will use the correct SPIRAM (8 MB for esp32s3). This will ensure we have the sufficient RAM to allocate feature data (image data).