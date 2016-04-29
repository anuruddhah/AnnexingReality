using UnityEngine;
using System.Collections;

namespace AnnexR{
	public class AnnexRObject : MonoBehaviour {
		public int objectPref = 10;	
		public bool allowJumping = false;

		public AnnexRObject(){
		}

		public void mprint(){
			print ("objectPref " + objectPref.ToString() +
			       "allowJumping" + allowJumping.ToString());
		}
	}
}