// Frida script to intercept all TextOut calls from Orbiter's MFD rendering
// Groups text by HDC (drawing surface) to distinguish MFD panels

var gdi32 = Process.getModuleByName('gdi32.dll');
var pTextOutA = gdi32.getExportByName('TextOutA');

function readLatin1(ptr, len) {
    var buf = ptr.readByteArray(len);
    var bytes = new Uint8Array(buf);
    var str = '';
    for (var i = 0; i < bytes.length; i++) {
        str += String.fromCharCode(bytes[i]);
    }
    return str;
}

if (pTextOutA) {
    Interceptor.attach(pTextOutA, {
        onEnter: function(args) {
            try {
                var hdc = args[0].toUInt32();
                var x = args[1].toInt32();
                var y = args[2].toInt32();
                var len = args[4].toInt32();
                if (len > 0 && len < 256) {
                    var str = readLatin1(args[3], len);
                    send({hdc: hdc, x: x, y: y, text: str});
                }
            } catch(e) {
                console.log('TextOutA error: ' + e.message);
            }
        }
    });
    console.log('Hooked TextOutA');
}
