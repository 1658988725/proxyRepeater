var nvr = require('./build/Release/node_nvr_addon.node');

var data = [{
        url: "rtsp://admin:12345@10.196.230.149/000100",
        endpoint: "rtmp://127.0.0.1:1935/srs",
        stream: "stream1",
        password: "publish2016"
   }, {
	    rtspUrl: "rtsp://888888:888888@10.196.230.150/cam/realmonitor?channel=1&subtype=0&unicast=true&proto=Onvif",
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