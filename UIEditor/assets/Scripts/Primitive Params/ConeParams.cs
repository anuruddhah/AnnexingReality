using UnityEngine;
using System.Collections;
using System;

namespace AnnexR{
	namespace Primitives{
		public class ConeParams : BaseParams{

			public ConeParams(){
				anglePref = 0.2f;
				heightPref = 0.6f;
				ratioPref = 0.1f;
			}

			public float angle;
			public float height;

			public float angleMin;
			public float angleMax;
			public float heightMin;
			public float heightMax;

			public Vector3 center; // Apex?
			public Vector3 axisDirection;

			// To avoid random rotations around the main axis
			public Vector3 localRight;
			public Vector3 localForward;
			
			public float anglePref;
			public float heightPref;
			public float ratioPref;
		};
	}
}