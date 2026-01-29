// Frida script to intercept MFD text and provide button label access
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
                    send({type: 'text', hdc: hdc, x: x, y: y, text: str});
                }
            } catch(e) {
                console.log('TextOutA error: ' + e.message);
            }
        }
    });
    console.log('Hooked TextOutA');
}

// --- Orbiter API access for MFD buttons ---
// Find oapiMFDButtonLabel export (may be C++ mangled)

var orbiterModule = null;
var fnButtonLabel = null;
var fnSendMFDKey = null;
var fnProcessButton = null;

// Find the main Orbiter module
Process.enumerateModules().forEach(function(mod) {
    var name = mod.name.toLowerCase();
    if (name.indexOf('orbiter') >= 0 && name.endsWith('.exe')) {
        orbiterModule = mod;
    }
});

if (orbiterModule) {
    console.log('Orbiter module: ' + orbiterModule.name + ' at ' + orbiterModule.base);

    // Search exports for MFD button functions
    var exports = orbiterModule.enumerateExports();
    exports.forEach(function(exp) {
        if (exp.name.indexOf('oapiMFDButtonLabel') >= 0 ||
            exp.name.indexOf('MFDButtonLabel') >= 0) {
            console.log('Found button label: ' + exp.name + ' at ' + exp.address);
            // const char* oapiMFDButtonLabel(int mfd, int bt)
            fnButtonLabel = new NativeFunction(exp.address, 'pointer', ['int', 'int'], 'mscdecl');
        }
        if (exp.name.indexOf('oapiSendMFDKey') >= 0 ||
            exp.name.indexOf('SendMFDKey') >= 0) {
            console.log('Found send key: ' + exp.name + ' at ' + exp.address);
            // int oapiSendMFDKey(int mfd, DWORD key)
            fnSendMFDKey = new NativeFunction(exp.address, 'int', ['int', 'uint32'], 'mscdecl');
        }
        if (exp.name.indexOf('oapiProcessMFDButton') >= 0 ||
            exp.name.indexOf('ProcessMFDButton') >= 0) {
            console.log('Found process button: ' + exp.name + ' at ' + exp.address);
            // bool oapiProcessMFDButton(int mfd, int bt, int event)
            fnProcessButton = new NativeFunction(exp.address, 'bool', ['int', 'int', 'int'], 'mscdecl');
        }
    });

    if (!fnButtonLabel) {
        // Dump all exports containing 'MFD' for debugging
        console.log('Could not find oapiMFDButtonLabel. MFD-related exports:');
        exports.forEach(function(exp) {
            if (exp.name.toUpperCase().indexOf('MFD') >= 0) {
                console.log('  ' + exp.name);
            }
        });
    }
} else {
    console.log('Could not find Orbiter module. Modules:');
    Process.enumerateModules().forEach(function(mod) {
        console.log('  ' + mod.name);
    });
}

// Handle RPC calls from Python
var handlers = {
    buttons: function(mfdId) {
        if (!fnButtonLabel) return {error: 'oapiMFDButtonLabel not found'};
        var labels = [];
        for (var i = 0; i < 12; i++) {
            var ptr = fnButtonLabel(mfdId, i);
            if (!ptr.isNull()) {
                labels.push({index: i, side: i < 6 ? 'L' : 'R', label: ptr.readUtf8String()});
            }
        }
        return {mfd: mfdId, buttons: labels};
    },
    press: function(mfdId, bt) {
        if (!fnProcessButton) return {error: 'oapiProcessMFDButton not found'};
        // PANEL_MOUSE_LBDOWN = 1
        var result = fnProcessButton(mfdId, bt, 1);
        return {mfd: mfdId, button: bt, result: result};
    }
};

recv('rpc', function onRpc(message) {
    var cmd = message.cmd;
    var args = message.args || [];
    var result;
    try {
        if (handlers[cmd]) {
            result = handlers[cmd].apply(null, args);
        } else {
            result = {error: 'Unknown command: ' + cmd};
        }
    } catch(e) {
        result = {error: e.message};
    }
    send({type: 'rpc_result', cmd: cmd, result: result});
    recv('rpc', onRpc);
});
