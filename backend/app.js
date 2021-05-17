const express = require('express');
const app = express();
const cors = require('cors');
const { exec, spawn } = require('child_process');
const sh = spawn('bash');
const serveIndex = require('serve-index');
const path = require("path");
const { dir } = require('console');
const ip = require("ip");

app.use(cors());
app.use(express.static(__dirname + "/dist/"))

app.get('/', function(req, res) {
    res.sendFile(__dirname + "/dist/index.html")
});


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
    res.send('Robotont service restart')
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
    res.send('Robotont service start')
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
    res.send('Robotont service stop')
});

let dirPath = path.join(__dirname, "../../");
app.use('/files', express.static(dirPath), serveIndex(dirPath, {icons: true}));

app.listen(3000, function() {
    console.log("*************************************")
    console.log("Web app running at: " + ip.address() + ":3000")
    console.log("running at port 3000");
    console.log("*************************************")
});

