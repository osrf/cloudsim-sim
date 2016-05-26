'use strict'

//const pub = require('./.publickey.js').publicKey
const jwt = require('jsonwebtoken')
const dotenv = require('dotenv')
const NodeRSA = require('node-rsa')

// read the environment variables
dotenv.config()
const publicKey = process.env.CLOUDSIM_AUTH_PUB_KEY
const privateKey = process.env.CLOUDSIM_AUTH_PRIV_KEY
const authUrl = process.env.CLOUDSIM_AUTH_URL


// generates keys that can be used with jsonwebtoken
exports.generateKeys = function() {
  var key = new NodeRSA({b: 512, e: 5});

    key.setOptions({
        encryptionScheme: {
        scheme: 'pkcs1',
        label: 'Optimization-Service'
    },
    signingScheme: {
        saltLength: 25
    }
  });

  return {
        "private" : key.exportKey('pkcs1-private-pem'),
        "public"  : key.exportKey('pkcs8-public-pem')
  };
}


// sign a token. This is done by the server
// the private key is necessary
// the private key should only be on the auth server
exports.signToken = function (data, cb) {
  jwt.sign(data, privateKey, { algorithm: 'RS256' }, cb)
}

// verify a token... requires the public key of the server in the .env file
exports.verifyToken = function(token, cb) {
  jwt.verify(token, publicKey,  {algorithms: ['RS256']}, cb)
}


// quick test of the functionality
exports.test = function() {
  console.log("\npub=[" +  publicKey + "]\n\npriv=[" + privateKey + "]\n\nurl", authUrl)
  exports.signToken({"success" : true }, (token) => {
    console.log('signed: ' + token)
    console.log('\n\nverify...')
    exports.verifyToken(token, console.log)
  })
}



