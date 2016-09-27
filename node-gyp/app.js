var nvr = require('./build/Release/node_nvr_addon.node');

var data = [{
	rtspUrl: "rtsp://127.0.0.1:8554/cam",
    endpoint: "rtmp://127.0.0.1:1935/srs",
    password: "publish2016"
}, {
	    url: "rtsp://10.196.230.150/cam/realmonitor?channel=1&subtype=0",
		rtspUsername: "888888",
		rtspPassword: "888888",
	    rtspUseTcp: 0,
        endpoint: "rtmp://127.0.0.1:1935/srs",
        stream: "stream2",
        password: "publish2016"
}];

nvr.init(JSON.stringify(data), function(err, msg){
	if(err) {
		console.error(msg);
		return;
	}
	nvr.start();
});