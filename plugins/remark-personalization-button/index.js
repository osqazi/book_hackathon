module.exports = function plugin() {
  return function transformer(tree, file) {
    // Only add the button to doc files, not other MDX content
    if (file.path.includes('docs/')) {
      // Create the MDX element node for the personalization button
      const buttonNode = {
        type: 'mdxJsxFlowElement',
        name: 'div',
        attributes: [
          {
            type: 'mdxJsxAttribute',
            name: 'className',
            value: 'doc-personalization-button-wrapper'
          }
        ],
        children: [
          {
            type: 'mdxJsxFlowElement',
            name: 'DocPersonalizationButton',
            attributes: [],
            children: []
          }
        ]
      };

      // Add import statement for the component
      const importNode = {
        type: 'mdxjsEsm',
        value: '',
        data: {
          estree: {
            type: 'Program',
            body: [
              {
                type: 'ImportDeclaration',
                specifiers: [
                  {
                    type: 'ImportSpecifier',
                    imported: { type: 'Identifier', name: 'DocPersonalizationButton' },
                    local: { type: 'Identifier', name: 'DocPersonalizationButton' }
                  }
                ],
                source: { type: 'Literal', value: '@site/src/components/DocPersonalizationButton' }
              }
            ],
            sourceType: 'module'
          }
        }
      };

      // Insert the import at the beginning of the tree
      tree.children.unshift(importNode);

      // Find the first h1 heading by manually traversing children
      let insertIndex = -1;
      for (let i = 0; i < tree.children.length; i++) {
        const node = tree.children[i];
        if (node.type === 'heading' && node.depth === 1) {
          insertIndex = i + 1;
          break;
        }
      }

      if (insertIndex !== -1) {
        tree.children.splice(insertIndex, 0, buttonNode);
      } else {
        // If no h1 found, insert at the beginning after imports
        tree.children.splice(1, 0, buttonNode);
      }
    }
  };
};
