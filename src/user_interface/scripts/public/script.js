var socket = io();

$(document).ready(function() {

    socket.on('packml_status', function(status){
        document.getElementById("state").innerHTML = status.state.val;
    });
    
    socket.on('packml_stats', function(msg){
        $('#availability-progress').progress({
            percent: msg.stats.availability * 100
        });
    });
});