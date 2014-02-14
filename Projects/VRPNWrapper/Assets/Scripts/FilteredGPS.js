var source : GameObject;
var filter : float = 0.5;
private var lastPosition : Vector3;

function Start() {
	lastPosition = source.transform.localPosition;
}

function LateUpdate () {
	transform.position = filter*source.transform.localPosition + (1-filter)*transform.position;
	transform.position.y = 0.0;
}