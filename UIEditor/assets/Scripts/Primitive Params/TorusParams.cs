using UnityEngine;
using System.Collections;
using System;

namespace AnnexR{
	namespace Primitives{
		public class TorusParams : BaseParams{

			public TorusParams(){
				minRadiusPref = 0.0f;
				maxRadiusPref = 1.0f;
				ratioPref = 0.0f;
			}

			public float minRadius;
			public float maxRadius;

			public float minRadiusMin;
			public float minRadiusMax;
			public float maxRadiusMin;
			public float maxRadiusMax;

			public Vector3 center;
			public Vector3 axisDirection;

			// To avoid random rotations around the main axis
			public Vector3 localRight;
			public Vector3 localForward;
			
			public float minRadiusPref;
			public float maxRadiusPref;
			public float ratioPref;
		};
	}
}