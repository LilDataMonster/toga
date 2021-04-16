module.exports = {
  devServer: {
    proxy: {
      '/api': {
        target: 'http://lildatamonster.local:80',
        changeOrigin: true,
        ws: true
      }
    }
  }
}
