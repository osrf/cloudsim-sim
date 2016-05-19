'use strict'

const jwt = require('jsonwebtoken')
const NodeRSA = require('node-rsa')

function generateKeys(){
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

var keys = generateKeys()

// console.log(keys)

// make them dotenv friendly
const privEnv = keys.private.replace( new RegExp( "\n", "g" ),"\\n")
const pubEnv  = keys.public.replace( new RegExp( "\n", "g" ),"\\n")


// console.log('\n\npriv:\n' + keys.private)
// console.log('\n\npub:\n' + keys.public)

// console.log("\n\n")
//console.log('#privEnv [' + privEnv + ']')
//console.log('#pubEnv [' + pubEnv + ']')

console.log('CLOUDSIM_AUTH_PRIV_KEY="' + privEnv + '"')
console.log('CLOUDSIM_AUTH_PUB_KEY="' + pubEnv + '"')
console.log('CLOUDSIM_AUTH_URL=https://x.x.x.x:5050')




