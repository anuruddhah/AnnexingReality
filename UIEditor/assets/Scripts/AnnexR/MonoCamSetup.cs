using UnityEngine;
using System.Collections;

public class MonoCamSetup : MonoBehaviour {
	WebCamTexture camTexture;

	void Start () {
		Screen.SetResolution(1280, 720, true);
		GUITexture backTexture = gameObject.AddComponent<GUITexture>();
		backTexture.pixelInset = new Rect(0, 0, 1280, 720);
		WebCamDevice[] webcams = WebCamTexture.devices;
		camTexture = new WebCamTexture(webcams[1].name, 1280, 720, 15);
		backTexture.texture = camTexture;
		camTexture.Play();
	}

	void OnDestroy(){
		//if (camTexture.isPlaying) {
			Debug.Log("Stopping camera");
			camTexture.Stop ();
		//}
	}

}
