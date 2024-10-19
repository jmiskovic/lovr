#include "os.h"
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include "event/event.h"
#include <android/looper.h>
#include <android/sensor.h>
// This is probably bad, but makes things easier to build
#include <android_native_app_glue.c>

static struct {
  struct android_app* app;
  JNIEnv* jni;
  fn_quit* onQuit;
  fn_key* onKeyboardEvent;
  fn_text* onTextEvent;
  fn_permission* onPermissionEvent;
  ASensorEventQueue *sensor_event_queue;
  bool is_window_open;
  uint32_t width;
  uint32_t height;
  unsigned touch_count;
  fn_mouse_button* onMouseButton;
  fn_mouse_move* onMouseMove;
} state;

static void onAppCmd(struct android_app* app, int32_t cmd) {
  switch (cmd) {
    case APP_CMD_INIT_WINDOW:
      // The window is being shown, get it ready.
      state.is_window_open = true;
      state.width = ANativeWindow_getWidth(app->window);
      state.height = ANativeWindow_getHeight(app->window);
      __android_log_write(ANDROID_LOG_DEBUG, "LOVR", "Received onAppCmd APP_CMD_INIT_WINDOW");
      break;
    case APP_CMD_TERM_WINDOW:
    case APP_CMD_DESTROY:
      // The window is being hidden or closed, clean it up.
      state.onQuit();
      break;
    default:
      break;
  }
}

static int handleTouch(struct android_app* app, AInputEvent* event) {
    if (AInputEvent_getType(event) != AINPUT_EVENT_TYPE_MOTION) {
        return 0;
    }

    int32_t action = AMotionEvent_getAction(event);
    int32_t flags = action & AMOTION_EVENT_ACTION_MASK;
    CustomEvent eventData;

    switch (flags) {
        case AMOTION_EVENT_ACTION_DOWN: {
            // First contact, data is always at index 0
            int32_t slot = AMotionEvent_getPointerId(event, 0);
            float x = AMotionEvent_getX(event, 0);
            float y = AMotionEvent_getY(event, 0);
            float size = AMotionEvent_getSize(event, 0);

            strncpy(eventData.name, "touchpressed", MAX_EVENT_NAME_LENGTH - 1);
            eventData.count = 4;
            eventData.data[0].type = TYPE_NUMBER;
            eventData.data[0].value.number = slot;
            eventData.data[1].type = TYPE_NUMBER;
            eventData.data[1].value.number = x;
            eventData.data[2].type = TYPE_NUMBER;
            eventData.data[2].value.number = y;
            eventData.data[3].type = TYPE_NUMBER;
            eventData.data[3].value.number = size;
            lovrEventPush((Event) {
                .type = EVENT_CUSTOM,
                .data.custom = eventData
            });
            if (state.onMouseButton) {
                state.onMouseButton(0, true);
            }
            return 1;
        }
        case AMOTION_EVENT_ACTION_POINTER_DOWN: {
            int32_t pointer_index = (action & AMOTION_EVENT_ACTION_POINTER_INDEX_MASK)
                                  >> AMOTION_EVENT_ACTION_POINTER_INDEX_SHIFT;
            int32_t slot = AMotionEvent_getPointerId(event, pointer_index);
            float x = AMotionEvent_getX(event, pointer_index);
            float y = AMotionEvent_getY(event, pointer_index);
            float size = AMotionEvent_getSize(event, pointer_index);

            strncpy(eventData.name, "touchpressed", MAX_EVENT_NAME_LENGTH - 1);
            eventData.count = 4;
            eventData.data[0].type = TYPE_NUMBER;
            eventData.data[0].value.number = slot;
            eventData.data[1].type = TYPE_NUMBER;
            eventData.data[1].value.number = x;
            eventData.data[2].type = TYPE_NUMBER;
            eventData.data[2].value.number = y;
            eventData.data[3].type = TYPE_NUMBER;
            eventData.data[3].value.number = size;

            lovrEventPush((Event) {
                .type = EVENT_CUSTOM,
                .data.custom = eventData
            });
            return 1;
        }

        case AMOTION_EVENT_ACTION_MOVE: {
            size_t pointer_count = AMotionEvent_getPointerCount(event);
            strncpy(eventData.name, "touchmoved", MAX_EVENT_NAME_LENGTH - 1);
            eventData.count = 4;

            // Handle all active pointers in the event
            for (size_t i = 0; i < pointer_count; i++) {
                int32_t slot = AMotionEvent_getPointerId(event, i);
                float x = AMotionEvent_getX(event, i);
                float y = AMotionEvent_getY(event, i);
                float size = AMotionEvent_getSize(event, i);

                eventData.data[0].type = TYPE_NUMBER;
                eventData.data[0].value.number = slot;  // Use slot instead of index
                eventData.data[1].type = TYPE_NUMBER;
                eventData.data[1].value.number = x;
                eventData.data[2].type = TYPE_NUMBER;
                eventData.data[2].value.number = y;
                eventData.data[3].type = TYPE_NUMBER;
                eventData.data[3].value.number = size;

                lovrEventPush((Event) {
                    .type = EVENT_CUSTOM,
                    .data.custom = eventData
                });
            }
            if (state.onMouseMove) {
                state.onMouseMove(AMotionEvent_getX(event, 0), AMotionEvent_getY(event, 0));
            }
            return 1;
        }
        case AMOTION_EVENT_ACTION_UP: {
            // Last contact lifted, data is always at index 0
            int32_t slot = AMotionEvent_getPointerId(event, 0);
            float x = AMotionEvent_getX(event, 0);
            float y = AMotionEvent_getY(event, 0);
            float size = AMotionEvent_getSize(event, 0);

            strncpy(eventData.name, "touchreleased", MAX_EVENT_NAME_LENGTH - 1);
            eventData.count = 4;
            eventData.data[0].type = TYPE_NUMBER;
            eventData.data[0].value.number = slot;
            eventData.data[1].type = TYPE_NUMBER;
            eventData.data[1].value.number = x;
            eventData.data[2].type = TYPE_NUMBER;
            eventData.data[2].value.number = y;
            eventData.data[3].type = TYPE_NUMBER;
            eventData.data[3].value.number = size;
            lovrEventPush((Event) {
                .type = EVENT_CUSTOM,
                .data.custom = eventData
            });
            if (state.onMouseButton) {
                state.onMouseButton(0, false);
            }
            return 1;
        }

        case AMOTION_EVENT_ACTION_POINTER_UP: {
            int32_t pointer_index = (action & AMOTION_EVENT_ACTION_POINTER_INDEX_MASK)
                                  >> AMOTION_EVENT_ACTION_POINTER_INDEX_SHIFT;
            int32_t slot = AMotionEvent_getPointerId(event, pointer_index);
            float x = AMotionEvent_getX(event, pointer_index);
            float y = AMotionEvent_getY(event, pointer_index);
            float size = AMotionEvent_getSize(event, pointer_index);

            strncpy(eventData.name, "touchreleased", MAX_EVENT_NAME_LENGTH - 1);
            eventData.count = 4;
            eventData.data[0].type = TYPE_NUMBER;
            eventData.data[0].value.number = slot;
            eventData.data[1].type = TYPE_NUMBER;
            eventData.data[1].value.number = x;
            eventData.data[2].type = TYPE_NUMBER;
            eventData.data[2].value.number = y;
            eventData.data[3].type = TYPE_NUMBER;
            eventData.data[3].value.number = size;

            lovrEventPush((Event) {
                .type = EVENT_CUSTOM,
                .data.custom = eventData
            });
            return 1;
        }
    }
    return 0;
}


static int32_t onInputEvent(struct android_app* app, AInputEvent* event) {
  if (handleTouch(app, event)) {
    return 1;
  }

  if (AInputEvent_getType(event) != AINPUT_EVENT_TYPE_KEY || !state.onKeyboardEvent) {
    return 0;
  }

  os_button_action action;
  switch (AKeyEvent_getAction(event)) {
    case AKEY_EVENT_ACTION_DOWN: action = BUTTON_PRESSED; break;
    case AKEY_EVENT_ACTION_UP: action = BUTTON_RELEASED; break;
    default: return 0;
  }

  os_key key;
  switch (AKeyEvent_getKeyCode(event)) {
    case AKEYCODE_A: key = OS_KEY_A; break;
    case AKEYCODE_B: key = OS_KEY_B; break;
    case AKEYCODE_C: key = OS_KEY_C; break;
    case AKEYCODE_D: key = OS_KEY_D; break;
    case AKEYCODE_E: key = OS_KEY_E; break;
    case AKEYCODE_F: key = OS_KEY_F; break;
    case AKEYCODE_G: key = OS_KEY_G; break;
    case AKEYCODE_H: key = OS_KEY_H; break;
    case AKEYCODE_I: key = OS_KEY_I; break;
    case AKEYCODE_J: key = OS_KEY_J; break;
    case AKEYCODE_K: key = OS_KEY_K; break;
    case AKEYCODE_L: key = OS_KEY_L; break;
    case AKEYCODE_M: key = OS_KEY_M; break;
    case AKEYCODE_N: key = OS_KEY_N; break;
    case AKEYCODE_O: key = OS_KEY_O; break;
    case AKEYCODE_P: key = OS_KEY_P; break;
    case AKEYCODE_Q: key = OS_KEY_Q; break;
    case AKEYCODE_R: key = OS_KEY_R; break;
    case AKEYCODE_S: key = OS_KEY_S; break;
    case AKEYCODE_T: key = OS_KEY_T; break;
    case AKEYCODE_U: key = OS_KEY_U; break;
    case AKEYCODE_V: key = OS_KEY_V; break;
    case AKEYCODE_W: key = OS_KEY_W; break;
    case AKEYCODE_X: key = OS_KEY_X; break;
    case AKEYCODE_Y: key = OS_KEY_Y; break;
    case AKEYCODE_Z: key = OS_KEY_Z; break;
    case AKEYCODE_0: key = OS_KEY_0; break;
    case AKEYCODE_1: key = OS_KEY_1; break;
    case AKEYCODE_2: key = OS_KEY_2; break;
    case AKEYCODE_3: key = OS_KEY_3; break;
    case AKEYCODE_4: key = OS_KEY_4; break;
    case AKEYCODE_5: key = OS_KEY_5; break;
    case AKEYCODE_6: key = OS_KEY_6; break;
    case AKEYCODE_7: key = OS_KEY_7; break;
    case AKEYCODE_8: key = OS_KEY_8; break;
    case AKEYCODE_9: key = OS_KEY_9; break;

    case AKEYCODE_SPACE: key = OS_KEY_SPACE; break;
    case AKEYCODE_ENTER: key = OS_KEY_ENTER; break;
    case AKEYCODE_TAB: key = OS_KEY_TAB; break;
    case AKEYCODE_ESCAPE: key = OS_KEY_ESCAPE; break;
    case AKEYCODE_DEL: key = OS_KEY_BACKSPACE; break;
    case AKEYCODE_DPAD_UP: key = OS_KEY_UP; break;
    case AKEYCODE_DPAD_DOWN: key = OS_KEY_DOWN; break;
    case AKEYCODE_DPAD_LEFT: key = OS_KEY_LEFT; break;
    case AKEYCODE_DPAD_RIGHT: key = OS_KEY_RIGHT; break;
    case AKEYCODE_MOVE_HOME: key = OS_KEY_HOME; break;
    case AKEYCODE_MOVE_END: key = OS_KEY_END; break;
    case AKEYCODE_PAGE_UP: key = OS_KEY_PAGE_UP; break;
    case AKEYCODE_PAGE_DOWN: key = OS_KEY_PAGE_DOWN; break;
    case AKEYCODE_INSERT: key = OS_KEY_INSERT; break;
    case AKEYCODE_FORWARD_DEL: key = OS_KEY_DELETE; break;
    case AKEYCODE_F1: key = OS_KEY_F1; break;
    case AKEYCODE_F2: key = OS_KEY_F2; break;
    case AKEYCODE_F3: key = OS_KEY_F3; break;
    case AKEYCODE_F4: key = OS_KEY_F4; break;
    case AKEYCODE_F5: key = OS_KEY_F5; break;
    case AKEYCODE_F6: key = OS_KEY_F6; break;
    case AKEYCODE_F7: key = OS_KEY_F7; break;
    case AKEYCODE_F8: key = OS_KEY_F8; break;
    case AKEYCODE_F9: key = OS_KEY_F9; break;
    case AKEYCODE_F10: key = OS_KEY_F10; break;
    case AKEYCODE_F11: key = OS_KEY_F11; break;
    case AKEYCODE_F12: key = OS_KEY_F12; break;

    case AKEYCODE_GRAVE: key = OS_KEY_BACKTICK; break;
    case AKEYCODE_MINUS: key = OS_KEY_MINUS; break;
    case AKEYCODE_EQUALS: key = OS_KEY_EQUALS; break;
    case AKEYCODE_LEFT_BRACKET: key = OS_KEY_LEFT_BRACKET; break;
    case AKEYCODE_RIGHT_BRACKET: key = OS_KEY_RIGHT_BRACKET; break;
    case AKEYCODE_BACKSLASH: key = OS_KEY_BACKSLASH; break;
    case AKEYCODE_SEMICOLON: key = OS_KEY_SEMICOLON; break;
    case AKEYCODE_APOSTROPHE: key = OS_KEY_APOSTROPHE; break;
    case AKEYCODE_COMMA: key = OS_KEY_COMMA; break;
    case AKEYCODE_PERIOD: key = OS_KEY_PERIOD; break;
    case AKEYCODE_SLASH: key = OS_KEY_SLASH; break;

    case AKEYCODE_CTRL_LEFT: key = OS_KEY_LEFT_CONTROL; break;
    case AKEYCODE_SHIFT_LEFT: key = OS_KEY_LEFT_SHIFT; break;
    case AKEYCODE_ALT_LEFT: key = OS_KEY_LEFT_ALT; break;
    case AKEYCODE_META_LEFT: key = OS_KEY_LEFT_OS; break;
    case AKEYCODE_CTRL_RIGHT: key = OS_KEY_RIGHT_CONTROL; break;
    case AKEYCODE_SHIFT_RIGHT: key = OS_KEY_RIGHT_SHIFT; break;
    case AKEYCODE_ALT_RIGHT: key = OS_KEY_RIGHT_ALT; break;
    case AKEYCODE_META_RIGHT: key = OS_KEY_RIGHT_OS; break;

    case AKEYCODE_CAPS_LOCK: key = OS_KEY_CAPS_LOCK; break;
    case AKEYCODE_SCROLL_LOCK: key = OS_KEY_SCROLL_LOCK; break;
    case AKEYCODE_NUM_LOCK: key = OS_KEY_NUM_LOCK; break;

    default: return 0;
  }

  uint32_t scancode = AKeyEvent_getScanCode(event);
  bool repeat = AKeyEvent_getRepeatCount(event) > 0;

  state.onKeyboardEvent(action, key, scancode, repeat);

  // Text event
  if (action == BUTTON_PRESSED && state.onTextEvent) {
    jclass jKeyEvent = (*state.jni)->FindClass(state.jni, "android/view/KeyEvent");
    jmethodID jgetUnicodeChar = (*state.jni)->GetMethodID(state.jni, jKeyEvent, "getUnicodeChar", "(I)I");
    jmethodID jKeyEventInit = (*state.jni)->GetMethodID(state.jni, jKeyEvent, "<init>", "(II)V");
    jobject jevent = (*state.jni)->NewObject(state.jni, jKeyEvent, jKeyEventInit, AKEY_EVENT_ACTION_DOWN, AKeyEvent_getKeyCode(event));
    uint32_t codepoint = (*state.jni)->CallIntMethod(state.jni, jevent, jgetUnicodeChar, AKeyEvent_getMetaState(event));
    if (codepoint > 0) {
      state.onTextEvent(codepoint);
    }
    (*state.jni)->DeleteLocalRef(state.jni, jevent);
    (*state.jni)->DeleteLocalRef(state.jni, jKeyEvent);
  }

  return 1;
}

int main(int argc, char** argv);

void android_main(struct android_app* app) {
  state.touch_count = 0;
  state.app = app;
  (*app->activity->vm)->AttachCurrentThread(app->activity->vm, &state.jni, NULL);
  os_open_console();
  app->onAppCmd = onAppCmd;
  app->onInputEvent = onInputEvent;

    jobject activity = state.app->activity->clazz;
    jclass class = (*state.jni)->GetObjectClass(state.jni, activity);
    jmethodID requestExternalStoragePermission = (*state.jni)->GetMethodID(state.jni, class, "requestExternalStoragePermission", "()V");
    if (!requestExternalStoragePermission) {
      (*state.jni)->DeleteLocalRef(state.jni, class);
      if (state.onPermissionEvent) state.onPermissionEvent(OS_PERMISSION_EXTERNAL_STORAGE, false);
      return;
    }
    (*state.jni)->CallVoidMethod(state.jni, activity, requestExternalStoragePermission);

  main(0, NULL);
  (*app->activity->vm)->DetachCurrentThread(app->activity->vm);
}

bool os_init(void) {
  return true;
}

void os_destroy(void) {
  // There are two ways a quit can happen, which need to be handled slightly differently:
  // - If the system tells us to quit, we get an event with APP_CMD_DESTROY.  In response we push a
  //   QUIT event to lovr.event and main will eventually exit cleanly.  No other teardown necessary.
  // - If the app exits manually (e.g. lovr.event.quit), then Android thinks we're still running and
  //   the app is still in APP_CMD_RESUME.  We need to tell it to exit with ANativeActivity_Finish
  //   and poll for events until the app state changes, otherwise the app is left in a broken state.
  if (state.app->activityState == APP_CMD_RESUME) {
    state.onQuit = NULL;
    state.onKeyboardEvent = NULL;
    state.onTextEvent = NULL;
    state.onPermissionEvent = NULL;
    ANativeActivity_finish(state.app->activity);
    while (!state.app->destroyRequested) {
      os_poll_events();
    }
  }
  memset(&state, 0, sizeof(state));
}

const char* os_get_name(void) {
  return "Android";
}

uint32_t os_get_core_count(void) {
  return sysconf(_SC_NPROCESSORS_ONLN);
}

// To make regular printing work, a thread makes a pipe and redirects stdout and stderr to the write
// end of the pipe.  The read end of the pipe is forwarded to __android_log_write.
static struct {
  bool attached;
  int handles[2];
  pthread_t thread;
} thread_log;

static void* log_main(void* data) {
  int* fd = data;
  pipe(fd);
  dup2(fd[1], STDOUT_FILENO);
  dup2(fd[1], STDERR_FILENO);
  setvbuf(stdout, 0, _IOLBF, 0);
  setvbuf(stderr, 0, _IONBF, 0);
  ssize_t length;
  char buffer[1024];
  while ((length = read(fd[0], buffer, sizeof(buffer) - 1)) > 0) {
    buffer[length] = '\0';
    __android_log_write(ANDROID_LOG_DEBUG, "LOVR", buffer);
  }
  return 0;
}

void os_open_console(void) {
  if (!thread_log.attached) {
    pthread_create(&thread_log.thread, NULL, log_main, thread_log.handles);
    pthread_detach(thread_log.thread);
    thread_log.attached = true;
  }
}

#define NS_PER_SEC 1000000000ULL

double os_get_time(void) {
  struct timespec t;
  clock_gettime(CLOCK_MONOTONIC, &t);
  return (double) t.tv_sec + (t.tv_nsec / (double) NS_PER_SEC);
}

void os_sleep(double seconds) {
  seconds += .5e-9;
  struct timespec t;
  t.tv_sec = seconds;
  t.tv_nsec = (seconds - t.tv_sec) * NS_PER_SEC;
  while (nanosleep(&t, &t));
}

#define _JNI(PKG, X) Java_ ## PKG ## _ ## X
#define JNI(PKG, X) _JNI(PKG, X)

JNIEXPORT void JNICALL JNI(LOVR_JAVA_PACKAGE, Activity_lovrPermissionEvent)(JNIEnv* jni, jobject activity, jint permission, jboolean granted) {
  if (state.onPermissionEvent) {
    state.onPermissionEvent(permission, granted);
  }
}

void os_request_permission(os_permission permission) {
  if (permission == OS_PERMISSION_AUDIO_CAPTURE) {
    jobject activity = state.app->activity->clazz;
    jclass class = (*state.jni)->GetObjectClass(state.jni, activity);
    jmethodID requestAudioCapturePermission = (*state.jni)->GetMethodID(state.jni, class, "requestAudioCapturePermission", "()V");
    if (!requestAudioCapturePermission) {
      (*state.jni)->DeleteLocalRef(state.jni, class);
      if (state.onPermissionEvent) state.onPermissionEvent(OS_PERMISSION_AUDIO_CAPTURE, false);
      return;
    }
    (*state.jni)->CallVoidMethod(state.jni, activity, requestAudioCapturePermission);
  }
  if (permission == OS_PERMISSION_EXTERNAL_STORAGE) {
    jobject activity = state.app->activity->clazz;
    jclass class = (*state.jni)->GetObjectClass(state.jni, activity);
    jmethodID requestExternalStoragePermission = (*state.jni)->GetMethodID(state.jni, class, "requestExternalStoragePermission", "()V");
    if (!requestExternalStoragePermission) {
      (*state.jni)->DeleteLocalRef(state.jni, class);
      if (state.onPermissionEvent) state.onPermissionEvent(OS_PERMISSION_EXTERNAL_STORAGE, false);
      return;
    }
    (*state.jni)->CallVoidMethod(state.jni, activity, requestExternalStoragePermission);
  }
}

const char* os_get_clipboard_text(void) {
  return NULL;
}

void os_set_clipboard_text(const char* text) {
  //
}

void* os_vm_init(size_t size) {
  return mmap(NULL, size, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
}

bool os_vm_free(void* p, size_t size) {
  return !munmap(p, size);
}

bool os_vm_commit(void* p, size_t size) {
  return !mprotect(p, size, PROT_READ | PROT_WRITE);
}

bool os_vm_release(void* p, size_t size) {
  return !madvise(p, size, MADV_DONTNEED);
}

void os_thread_attach(void) {
  JNIEnv* jni;
  (*state.app->activity->vm)->AttachCurrentThread(state.app->activity->vm, &jni, NULL);
}

void os_thread_detach(void) {
  (*state.app->activity->vm)->DetachCurrentThread(state.app->activity->vm);
}

void os_poll_events(void) {
  if (!state.app->destroyRequested) {

    ASensorEvent event;
    while (ASensorEventQueue_getEvents(state.sensor_event_queue, &event, 1) > 0) {
      switch (event.type) {
        case ASENSOR_TYPE_ACCELEROMETER: {
          CustomEvent eventData;
          strncpy(eventData.name, "accel", MAX_EVENT_NAME_LENGTH - 1);
          eventData.count = 3;
          eventData.data[0].type = TYPE_NUMBER;
          eventData.data[1].type = TYPE_NUMBER;
          eventData.data[2].type = TYPE_NUMBER;
          eventData.data[0].value.number = event.vector.x;
          eventData.data[1].value.number = event.vector.y;
          eventData.data[2].value.number = event.vector.z;
          lovrEventPush((Event) {
            .type = EVENT_CUSTOM,
            .data.custom = eventData
          });
          break;
        }
        case ASENSOR_TYPE_GYROSCOPE: {
          CustomEvent eventData;
          strncpy(eventData.name, "gyro", MAX_EVENT_NAME_LENGTH - 1);
          eventData.count = 3;
          eventData.data[0].type = TYPE_NUMBER;
          eventData.data[1].type = TYPE_NUMBER;
          eventData.data[2].type = TYPE_NUMBER;
          eventData.data[0].value.number = event.vector.x;
          eventData.data[1].value.number = event.vector.y;
          eventData.data[2].value.number = event.vector.z;
          lovrEventPush((Event) {
            .type = EVENT_CUSTOM,
            .data.custom = eventData
          });
          break;
        }
        default: {
          break;
        }
      }
    }

    struct android_poll_source* source;
    int timeout = (state.app->window && state.app->activityState == APP_CMD_RESUME) ? 0 : -1;
    ALooper_pollOnce(timeout, NULL, NULL, (void**) &source);

    if (source) {
      source->process(state.app, source);
    }
  }
}

void os_on_quit(fn_quit* callback) {
  state.onQuit = callback;
}

void os_on_focus(fn_focus* callback) {
  //
}

void os_on_resize(fn_resize* callback) {
  //
}

void os_on_key(fn_key* callback) {
  state.onKeyboardEvent = callback;
}

void os_on_text(fn_text* callback) {
  state.onTextEvent = callback;
}

void os_on_mouse_button(fn_mouse_button* callback) {
  state.onMouseButton = callback;
}

void os_on_mouse_move(fn_mouse_move* callback) {
  state.onMouseMove = callback;
}

void os_on_mousewheel_move(fn_mousewheel_move* callback) {
  //
}

void os_on_permission(fn_permission* callback) {
  state.onPermissionEvent = callback;
}

bool os_window_open(const os_window_config* config) {
  ASensorManager *sensor_manager;
  const ASensor *accelerometer;
  const ASensor *gyroscope;
  #if __ANDROID_API__ >= 26
    __android_log_write(ANDROID_LOG_DEBUG, "LOVR", "API >= 26");
    sensor_manager = ASensorManager_getInstanceForPackage("org.lovr.app");
  #else
    __android_log_write(ANDROID_LOG_DEBUG, "LOVR", "API < 26");
    sensor_manager = ASensorManager_getInstance();
  #endif

  ALooper *looper = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
  state.sensor_event_queue = ASensorManager_createEventQueue(sensor_manager,
    looper, 0,
    NULL, NULL);
  accelerometer = ASensorManager_getDefaultSensor(sensor_manager, ASENSOR_TYPE_ACCELEROMETER);
  gyroscope = ASensorManager_getDefaultSensor(sensor_manager, ASENSOR_TYPE_GYROSCOPE);
  int32_t acc_min_delay =  1e6 / 60;  // in microseconds
  int32_t gyro_min_delay = 1e6 / 60;
  if (!sensor_manager) {
    __android_log_write(ANDROID_LOG_DEBUG, "LOVR", "Failed to get a sensor manager");
    return NULL;
  }
  if (accelerometer == NULL) {
    __android_log_write(ANDROID_LOG_DEBUG, "LOVR", "Failed to get a accel sensor");
  } else {
    ASensorEventQueue_enableSensor(state.sensor_event_queue, accelerometer);
    ASensorEventQueue_setEventRate(state.sensor_event_queue, accelerometer, acc_min_delay);
  }
  if (gyroscope == NULL) {
    __android_log_write(ANDROID_LOG_DEBUG, "LOVR", "Failed to get a gyro sensor");
  } else {
    ASensorEventQueue_enableSensor(state.sensor_event_queue, gyroscope);
    ASensorEventQueue_setEventRate(state.sensor_event_queue, gyroscope, gyro_min_delay);
  }
  int i= 0;
  while(!state.is_window_open && i < 100) {
    os_sleep(0.02);
    os_poll_events();
    __android_log_write(ANDROID_LOG_DEBUG, "LOVR", "waiting for window open");
    i++;
  };

  return true;
}

bool os_window_is_open(void) {
  return state.is_window_open;
}

uintptr_t os_get_android_window() {
  return (uintptr_t) state.app->window;
}

void os_window_get_size(uint32_t* width, uint32_t* height) {
  *width = state.width;
  *height = state.height;
}

float os_window_get_pixel_density(void) {
  return 1.f;
}

size_t os_get_home_directory(char* buffer, size_t size) {
  return 0;
}

size_t os_get_data_directory(char* buffer, size_t size) {
  const char* path = state.app->activity->externalDataPath;
  size_t length = strlen(path);
  if (length >= size) return 0;
  memcpy(buffer, path, length);
  buffer[length] = '\0';
  return length;
}

size_t os_get_working_directory(char* buffer, size_t size) {
  return getcwd(buffer, size) ? strlen(buffer) : 0;
}

size_t os_get_executable_path(char* buffer, size_t size) {
  ssize_t length = readlink("/proc/self/exe", buffer, size - 1);
  if (length >= 0) {
    buffer[length] = '\0';
    return length;
  } else {
    return 0;
  }
}

size_t os_get_bundle_path(char* buffer, size_t size, const char** root) {
  const char* customPath = "/sdcard/lovrproj01";
  // Check if the custom path exists
  struct stat st;
  if (stat(customPath, &st) == 0 && S_ISDIR(st.st_mode)) {
    size_t length = strlen(customPath);
    if (length >= size) return 0;
    memcpy(buffer, customPath, length);
    buffer[length] = '\0';
    *root = "";
    return length;
  }
  // Fallback to the original implementation
  jobject activity = state.app->activity->clazz;
  jclass class = (*state.jni)->GetObjectClass(state.jni, activity);
  jmethodID getPackageCodePath = (*state.jni)->GetMethodID(state.jni, class, "getPackageCodePath", "()Ljava/lang/String;");
  if (!getPackageCodePath) {
    (*state.jni)->DeleteLocalRef(state.jni, class);
    return 0;
  }

  jstring jpath = (*state.jni)->CallObjectMethod(state.jni, activity, getPackageCodePath);
  (*state.jni)->DeleteLocalRef(state.jni, class);
  if ((*state.jni)->ExceptionOccurred(state.jni)) {
    (*state.jni)->ExceptionClear(state.jni);
    return 0;
  }

  size_t length = (*state.jni)->GetStringUTFLength(state.jni, jpath);
  if (length >= size) return 0;

  const char* path = (*state.jni)->GetStringUTFChars(state.jni, jpath, NULL);
  memcpy(buffer, path, length);
  buffer[length] = '\0';
  (*state.jni)->ReleaseStringUTFChars(state.jni, jpath, path);
  *root = "/assets";
  return length;
}

void os_get_mouse_position(double* x, double* y) {
  *x = *y = 0.;
}

void os_set_mouse_mode(os_mouse_mode mode) {
  //
}

bool os_is_mouse_down(os_mouse_button button) {
  return false;
}

bool os_is_key_down(os_key key) {
  return false;
}

// Private, must be declared manually to use

void* os_get_java_vm(void) {
  return state.app->activity->vm;
}

void* os_get_jni_context(void) {
  return state.app->activity->clazz;
}

const char** os_vk_get_instance_extensions(uint32_t* count) {
  return *count = 0, NULL;
}

uint32_t os_vk_create_surface(void* instance, void** surface) {
  return -13; // VK_ERROR_UNKNOWN
}
