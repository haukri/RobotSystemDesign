
function initilizepackml() {

    var width = 1000, height = 600;

    var draw = SVG('packmldraw').size(width,height);
    draw.viewbox(0,0,width,height)
    //draw.rect(0,0,width,height);

    var background = draw.rect(width,height).fill('#dde3e1')


}