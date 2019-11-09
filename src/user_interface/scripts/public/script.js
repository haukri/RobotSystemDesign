var socket = io();

$(document).ready(function() {

    socket.on('packml_status', function(status){
        document.getElementById("state").innerHTML = status.state.val;
        resetpacmkdaigram();
        setpackmldiagram(status.state.val);


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
    //initilizepackml();

    var image = SVG('packmldraw');
    $.get('drawing.svg', function(contents) {
        var $tmp = $('svg', contents);
        image.svg($tmp.html());
        image.attr('viewBox', $tmp.attr('viewBox'));
        image.attr('width', $tmp.attr('width'));
        image.attr('height', $tmp.attr('height'));
    }, 'xml');




    // $('#packmldraw').hover(function() {
    //   //SVG.get('stopped').fill('blue');
    //     $('#stopped').css('fill',"blue")
    // }
    //);

});

function resetpacmkdaigram() {
    $('#stopped').css('fill',"#ffcc00")
    $('#idle').css('fill',"#ffcc00")
    $('#held').css('fill',"#ffcc00")
    $('#suspended').css('fill',"#ffcc00")
    $('#complete').css('fill',"#ffcc00")
    $('#aborted').css('fill',"#ffcc00")
    $('#execute').css('fill',"#0068ff")
    $('#reset').css('fill',"#00ff00")
    $('#start').css('fill',"#00ff00")
    $('#holding').css('fill',"#00ff00")
    $('#unhold').css('fill',"#00ff00")
    $('#suspending').css('fill',"#00ff00")
    $('#unsuspend').css('fill',"#00ff00")
    $('#completing').css('fill',"#00ff00")
    $('#aborting').css('fill',"#00ff00")
    $('#clear').css('fill',"#00ff00")
    $('#stopping').css('fill',"#00ff00")
}

function setpackmldiagram(state) {
    switch (state) {
        case 2:
            $('#stopped').css('fill',"#ff00ff")
            break;
        case 3:
            $('#starting').css('fill',"#ff00ff")
            break;
        case 4:
            $('#idle').css('fill',"#ff00ff")
            break;
        case 5:
            $('#suspended').css('fill',"#ff00ff")
            break;
        case 6:
            $('#execute').css('fill',"#ff00ff")
            break;
        case 7:
            $('#stopping').css('fill',"#ff00ff")
            break;
        case 8:
            $('#aborting').css('fill',"#ff00ff")
            break;
        case 9:
            $('#aborted').css('fill',"#ff00ff")
            break;
        case 10:
            $('#holding').css('fill',"#ff00ff")
            break;
        case 11:
            $('#held').css('fill',"#ff00ff")
            break;
        case 100:
            $('#reset').css('fill',"#ff00ff")
            break;
        case 101:
            $('#suspending').css('fill',"#ff00ff")
            break;
        case 102:
            $('#unsuspend').css('fill',"#ff00ff")
            break;
        case 103:
            $('#clear').css('fill',"#ff00ff")
            break;
        case 104:
            $('#unhold').css('fill',"#ff00ff")
            break;
        case 105:
            $('#completing').css('fill',"#ff00ff")
            break;
        case 106:
            $('#complete').css('fill',"#ff00ff")
            break;

    }
}



function changetab(evt, tabName) {
    var i, tabcontent, tablinks;
    tabcontent = document.getElementsByClassName("tabcontent");
    for (i = 0; i < tabcontent.length; i++) {
        tabcontent[i].style.display = "none";
    }
    tablinks = document.getElementsByClassName("item");
    for (i = 0; i < tablinks.length; i++) {
        tablinks[i].className = tablinks[i].className.replace(" active", "");
    }
    document.getElementById(tabName).style.display = "block";
    evt.currentTarget.className += " active";
}
document.getElementById("defaultOpen").click();


function packmlCommand(command) {
    console.log(command);
    socket.emit('packml_command',{command:command})
}