var socket = io();

socket.on('packml_status', function(status){
    document.getElementById("state").innerHTML = status.state.val;
});

socket.on('packml_stats', function(msg){
    document.getElementById("availability").innerHTML = msg.stats.availability;
});
