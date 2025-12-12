// plugins/docusaurus-auth/index.js
// Docusaurus plugin for Better-Auth integration

const path = require('path');

/** @type {import('@docusaurus/types').Plugin} */
module.exports = function (context, options) {
  const { siteConfig } = context;
  const config = {
    // Default options
    ...options,
  };

  return {
    name: 'docusaurus-auth-plugin',

    // Extend the webpack configuration to include our auth components
    configureWebpack: (config, isServer, utils) => {
      return {
        resolve: {
          alias: {
            '@auth': path.resolve(__dirname, '../src/auth'),
            '@models': path.resolve(__dirname, '../src/models'),
          },
        },
      };
    },

    // Inject auth context at the root of the app
    getClientModules() {
      return [path.resolve(__dirname, './src/AuthProviderWrapper')];
    },

    // Add auth-related scripts
    injectHtmlTags() {
      return {
        headTags: [
          {
            tagName: 'script',
            attributes: {
              src: '/auth-client.js',
              type: 'module',
            },
          },
        ],
      };
    },
  };
};

module.exports.validateOptions = ({ options }) => {
  // Return options as-is, no validation needed for this plugin
  return options;
};