package @ANDROID_PACKAGE@;

import android.Manifest;
import android.app.NativeActivity;
import android.content.pm.PackageManager;
import android.os.Build;

public class Activity extends NativeActivity {
  static {
    System.loadLibrary("openxr_loader");
    System.loadLibrary("lovr");
  }

  protected native void lovrPermissionEvent(int permission, boolean granted);

  @Override
  public void onRequestPermissionsResult(int requestCode, String[] permissions, int[] grantResults) {
    if (requestCode == 1) { // Audio capture permission request
      if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
        lovrPermissionEvent(0, true);
      } else {
        lovrPermissionEvent(0, false);
      }
    } else if (requestCode == 2) { // External storage permission request
      if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
        lovrPermissionEvent(1, true);
      } else {
        lovrPermissionEvent(1, false);
      }
    }
  }

  private void requestAudioCapturePermission() {
    if (checkSelfPermission(Manifest.permission.RECORD_AUDIO) != PackageManager.PERMISSION_GRANTED) {
      requestPermissions(new String[] { Manifest.permission.RECORD_AUDIO }, 1);
    } else {
      lovrPermissionEvent(0, true);
    }
  }

  private void requestExternalStoragePermission() {
    if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.M) {
      if (checkSelfPermission(Manifest.permission.READ_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED) {
        requestPermissions(new String[] { Manifest.permission.READ_EXTERNAL_STORAGE }, 2);
      } else {
        lovrPermissionEvent(1, true);
      }
    } else {
      lovrPermissionEvent(1, true); // on Android M and below the granting should be automatic
    }
  }
}
