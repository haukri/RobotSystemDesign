var socket = io();

var packMLStates = {};
packMLStates[2] = 'STOPPED'
packMLStates[3] = 'STARTING'
packMLStates[4] = 'IDLE'
packMLStates[5] = 'SUSPENDED'
packMLStates[6] = 'EXECUTE'
packMLStates[7] = 'STOPPING'
packMLStates[8] = 'ABORTING'
packMLStates[9] = 'ABORTED'
packMLStates[10] = 'HOLDING'
packMLStates[11] = 'HELD'
packMLStates[100] = 'RESETTING'
packMLStates[101] = 'SUSPENDING'
packMLStates[102] = 'UNSUSPENDING'
packMLStates[103] = 'CLEARING'
packMLStates[104] = 'UNHOLDING'
packMLStates[105] = 'COMPLETING'
packMLStates[106] = 'COMPLETE'


$(document).ready(function () {

    socket.on('packml_status', function (status) {
        document.getElementById("state").innerHTML = packMLStates[status.state.val];
        resetpacmkdaigram();
        setpackmldiagram(status.state.val);


    });

    socket.on('order_status', function (status) {
        document.getElementById("order-number").innerHTML = status.order_number;
        document.getElementById("red-brick").innerHTML = status.red_amount;
        document.getElementById("blue-brick").innerHTML = status.blue_amount;
        document.getElementById("yellow-brick").innerHTML = status.yellow_amount;

    });

    socket.on('main_control_status', function (status) {
        document.getElementById("main-control-state").innerHTML = status.data;
    });

    socket.on('packml_stats', function (msg) {
        $('#availability-progress').progress({
            percent: msg.stats.availability * 100
        });
        $('#quality-progress').progress({
            percent: msg.stats.quality * 100
        });
        $('#performance-progress').progress({
            percent: msg.stats.performance * 100 > 100 ? 100 : msg.stats.performance * 100
        });
        $('#oee-progress').progress({
            percent: msg.stats.overall_equipment_effectiveness * 100
        });
        document.getElementById("uptime").innerHTML = Math.floor(msg.stats.duration.data.secs / 60.0 / 60) + " hours " + Math.floor(msg.stats.duration.data.secs / 60.0) % 60 + " minutes";
    });

    socket.on('feeder_empty', function (feederEmpty) {
        if (feederEmpty) {
            $('.ui.modal').modal({
                closable: false,
                onApprove: function () {
                    socket.emit('feeder_full', {});
                }
            }).modal('show');
        }
    });

    socket.on('orders_per_hour', function (ordersPerHour) {
        document.getElementById("order-per-hour").innerHTML = ordersPerHour;
    });

    /*socket.on('packml_stats', function(msg){
        document.getElementById( "availability").innerHTML = msg.stats.availability
    });*/
    //initilizepackml();

    var image = SVG('packmldraw');
    $.get('drawing.svg', function (contents) {
        var $tmp = $('svg', contents);
        image.svg($tmp.html());
        image.attr('viewBox', $tmp.attr('viewBox'));
        image.attr('width', "100%"); //$tmp.attr('width'));
        image.attr('height', "50vh"); //$tmp.attr('height'));
    }, 'xml');




    // $('#packmldraw').hover(function() {
    //   //SVG.get('stopped').fill('blue');
    //     $('#stopped').css('fill',"blue")
    // }
    //);

});

function resetpacmkdaigram() {
    $('#stopped').css('fill', "#ffcc00")
    $('#idle').css('fill', "#ffcc00")
    $('#held').css('fill', "#ffcc00")
    $('#suspended').css('fill', "#ffcc00")
    $('#complete').css('fill', "#ffcc00")
    $('#aborted').css('fill', "#ffcc00")
    $('#execute').css('fill', "#0068ff")
    $('#reset').css('fill', "#00ff00")
    $('#start').css('fill', "#00ff00")
    $('#holding').css('fill', "#00ff00")
    $('#unhold').css('fill', "#00ff00")
    $('#suspending').css('fill', "#00ff00")
    $('#unsuspend').css('fill', "#00ff00")
    $('#completing').css('fill', "#00ff00")
    $('#aborting').css('fill', "#00ff00")
    $('#clear').css('fill', "#00ff00")
    $('#stopping').css('fill', "#00ff00")
}

function increaseOrderPerHour() {
    socket.emit('increase_order_per_hour', {  })
}

function decreaseOrderPerHour() {
    socket.emit('decrease_order_per_hour', {  })
}

function setpackmldiagram(state) {
    switch (state) {
        case 2:
            $('#stopped').css('fill', "#ff00ff")
            break;
        case 3:
            $('#starting').css('fill', "#ff00ff")
            break;
        case 4:
            $('#idle').css('fill', "#ff00ff")
            break;
        case 5:
            $('#suspended').css('fill', "#ff00ff")
            break;
        case 6:
            $('#execute').css('fill', "#ff00ff")
            break;
        case 7:
            $('#stopping').css('fill', "#ff00ff")
            break;
        case 8:
            $('#aborting').css('fill', "#ff00ff")
            break;
        case 9:
            $('#aborted').css('fill', "#ff00ff")
            break;
        case 10:
            $('#holding').css('fill', "#ff00ff")
            break;
        case 11:
            $('#held').css('fill', "#ff00ff")
            break;
        case 100:
            $('#reset').css('fill', "#ff00ff")
            break;
        case 101:
            $('#suspending').css('fill', "#ff00ff")
            break;
        case 102:
            $('#unsuspend').css('fill', "#ff00ff")
            break;
        case 103:
            $('#clear').css('fill', "#ff00ff")
            break;
        case 104:
            $('#unhold').css('fill', "#ff00ff")
            break;
        case 105:
            $('#completing').css('fill', "#ff00ff")
            break;
        case 106:
            $('#complete').css('fill', "#ff00ff")
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
    socket.emit('packml_command', { command: command })
}