var shell = require('shelljs');

function shutdown() {
    return shell.exec('sudo poweroff')
}

function rosRestart() {
    return shell.exec('sudo systemctl restart robotont.service')
}

function rosStart() {
    return shell.exec('sudo systemctl start robotont.service')
}

function rosStop() {
    return shell.exec('sudo systemctl stop robotont.service')
}

function shellOutput(command) {
    return shell.exec(command).stdout
}

module.exports = {
    shutdown: shutdown,
    rosRestart: rosRestart,
    rosStart: rosStart,
    rosStop: rosStop,
    shellOutput: shellOutput
};