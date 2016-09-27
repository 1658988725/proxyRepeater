var nvr = require('./build/Release/node_nvr_addon.node');

var data = [{
	rtspUrl: "rtsp://10.196.230.149/000100",
    endpoint: "rtmp://127.0.0.1:1935/srs",
    password: "publish2016"
}, {
	rtspUrl: "rtsp://10.196.230.150/cam/realmonitor?channel=1&subtype=0",
	rtspUsername: "888888",
	rtspPassword: "888888",
	rtspUseTcp: 0,
    endpoint: "rtmp://127.0.0.1:1935/srs",
    stream: "stream1",
    password: "publish2016"
}];

nvr.init(JSON.stringify(data), function(err, msg){
	if(err) {
		console.error(msg);
		return;
	}
	nvr.start();
});