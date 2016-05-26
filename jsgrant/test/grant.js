'use strict'

const should = require('should')
const grantjs = require('../index')
const model = require('../model')
const token = require('../token')

const keys = token.generateKeys()

let mToken = {username}

describe('<Unit Test grant>', function() {

  before(function(done) {
      console.log('erasing data before')
      model.clearDb()
      token.signToken({username: 'me'}, (e)=>{
        done()
      })
  })

  describe('adding and sharing a resource:', function(){
    it('should have an empty db', (done) => {
      grantjs.init(keys.public)
      grantjs.setResource()
    })

    it('should have a toaster', (done) => {

    })

    it('should be possible to share the toaster with joe', (done) => {

    })

    it('should be possible to revoke the toaster for joe', (done) => {
    })

    it('should be possible to remove the toaster', (done) => {
    })

  })
})
