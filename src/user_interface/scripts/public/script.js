var socket = io();

$(document).ready(function() {

    socket.on('packml_status', function(status){
        document.getElementById("state").innerHTML = status.state.val;
    });
    
    socket.on('packml_stats', function(msg){
        $('#availability-progress').progress({
            percent: msg.stats.availability * 100
        });
        document.getElementById( "availability2").innerHTML = msg.stats.availability;
        document.getElementById( "performance").innerHTML = msg.stats.performance;
        document.getElementById( "quality").innerHTML = msg.stats.quality;
        document.getElementById( "oee").innerHTML = msg.stats.overall_equipment_effectiveness;
        document.getElementById( "duration").innerHTML = msg.stats.duration.data.secs;
        document.getElementById( "heldduration").innerHTML = msg.stats.held_duration.data.secs;
        document.getElementById( "suspendedduration").innerHTML = msg.stats.susp_duration.data.secs;
        document.getElementById( "idleduration").innerHTML = msg.stats.idle_duration.data.secs;
        document.getElementById( "executeduration").innerHTML = msg.stats.exe_duration.data.secs;
    });
    /*socket.on('packml_stats', function(msg){
        document.getElementById( "availability").innerHTML = msg.stats.availability
    });*/

});