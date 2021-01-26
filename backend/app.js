const express = require('express');
const app = express();
const ws = require('ws');
const cors = require('cors');
const { exec, spawn } = require('child_process');
//const functions = require('./public/javascripts/functions.js');
const sh = spawn('bash');

app.use(cors());

// Set up a headless websocket server that prints any
// events that come in.
const wsServer = new ws.Server({ noServer: true });

var connect = function() {
    wsServer.on('connection', socket => {
        socket.on('message', message => 
            socket.send(sh.stdout.on(message)));
    });
}

app.post('/shutdown', function (req, res) {
    exec("sudo poweroff", (error, stdout, stdrr) => {
        if (error) {
            console.error(`error: ${error.message}`);
        }

        if (stdrr) {
            console.error(`stderr: ${stdrr}`)
        }

        console.log(`stdout:\n${stdout}`);
    });
    res.send('Success! Shutting down')
});

//'sudo systemctl restart robotont.service'
app.post('/rosRestart', function (req, res) {
    exec("sudo systemctl restart robotont.service", (error, stdout, stdrr) => {
        if (error) {
            console.error(`error: ${error.message}`);
        }

        if (stdrr) {
            console.error(`stderr: ${stdrr}`)
        }

        console.log(`stdout:\n${stdout}`);
    });
    res.send('Ros restart')
});

app.post('/rosStart', function (req, res) {
    exec("sudo systemctl start robotont.service", (error, stdout, stdrr) => {
        if (error) {
            console.error(`error: ${error.message}`);
        }

        if (stdrr) {
            console.error(`stderr: ${stdrr}`)
        }

        console.log(`stdout:\n${stdout}`);
    });
    res.send('Ros start')
});

app.post('/rosStop', function (req, res) {
    exec("sudo systemctl stop robotont.service", (error, stdout, stdrr) => {
        if (error) {
            console.error(`error: ${error.message}`);
        }

        if (stdrr) {
            console.error(`stderr: ${stdrr}`)
        }

        console.log(`stdout:\n${stdout}`);
    });
    res.send('Ros stop')
});

app.get('/shell', function(req, res) {
    var output = functions.shellOutput('rostopic list')
    res.send(output)
});

const server = app.listen(3000, function() {
    console.log("running at port 3000");
});

server.on('upgrade', (request, socket, head) => {
    wsServer.handleUpgrade(request, socket, head, socket => {
        wsServer.emit('connection', socket, request);
    });
});

connect();
