# Humanoid Robotics Book RAG System

This repository contains a complete Retrieval-Augmented Generation (RAG) system for the Humanoid Robotics Book, featuring a chatbot that can answer questions about the book content with source citations.

## 🚀 System Architecture

The RAG system consists of:

1. **Frontend**: Docusaurus-based documentation site with an integrated chat widget
2. **Backend**: FastAPI server with RAG capabilities
3. **Vector Database**: Qdrant Cloud for document storage and retrieval
4. **Language Model**: OpenAI GPT for response generation
5. **Database**: Neon Serverless Postgres for conversation history

## 📋 Features

- **Context-Aware Chat**: Ask questions about humanoid robotics concepts
- **Text Selection**: Select text on any page to provide context for your questions
- **Source Citations**: Responses include references to specific book sections
- **Conversation History**: Maintains session context with Neon Postgres
- **Responsive UI**: Chat widget works on all devices

## 🛠️ Tech Stack

- **Frontend**: React, TypeScript, Docusaurus
- **Backend**: Python, FastAPI, Uvicorn
- **Vector Database**: Qdrant Cloud
- **Language Model**: OpenAI GPT-3.5/4
- **Database**: Neon Serverless Postgres
- **Embeddings**: OpenAI text-embedding-ada-002

## 🚀 Quick Start

### Prerequisites

- Python 3.8+
- Node.js 16+
- OpenAI API key
- Neon Postgres database URL

### Installation

1. **Clone the repository**:
   ```bash
   git clone <repository-url>
   cd humanoid-robotics-book
   ```

2. **Install Python dependencies**:
   ```bash
   cd backend
   pip install -r requirements.txt
   ```

3. **Install Node.js dependencies** (for Docusaurus):
   ```bash
   npm install
   ```

4. **Set up environment variables**:
   ```bash
   cp .env .env.local
   # Edit .env.local and add your API keys
   ```

### Environment Variables

Create a `.env.local` file in the project root with the following variables:

```env
OPENAI_API_KEY=your_openai_api_key_here
NEON_DATABASE_URL=your_neon_database_url_here
QDRANT_URL=https://9751179e-b403-46d5-bf9e-d46ef1603cb7.us-east4-0.gcp.cloud.qdrant.io
QDRANT_API_KEY=eyJhbGciOiJIUzI1NiIsInR5cCI6IkpXVCJ9.eyJhY2Nlc3MiOiJtIn0.BRl-MTLWa4X8xZDMvz8e2KEbz53V5Rt4_uT78IBuWH8
```

## 🏃‍♂️ Running the System

### 1. Start the Backend Server

```bash
cd backend
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

The backend will automatically initialize the Qdrant collection and vectorize the book content on first run.

### 2. Start the Docusaurus Frontend

In a new terminal:

```bash
npm run start
```

The documentation site will be available at `http://localhost:3000`.

## 🔧 Configuration

### Backend Configuration

The backend API endpoints are:

- `GET /health` - Health check
- `POST /chat` - Chat with RAG system
- `GET /search` - Search documents
- `GET /sessions/{session_id}/messages` - Get session messages

### Frontend Integration

The chat widget is automatically integrated into all Docusaurus pages through the Layout override in `src/theme/Layout.js`.

## 📚 Using the RAG System

1. **Start both servers** (backend and frontend)
2. **Navigate to the documentation site**
3. **Click the chat icon** (💬) in the bottom-right corner
4. **Ask questions** about humanoid robotics
5. **Select text** on any page to provide context for your questions

### Text Selection Feature

- Select any text on the page
- The selected text will appear in the chat widget
- Ask questions related to the selected text for context-aware responses
- Clear the selection using the '×' button if needed

## 🗄️ Database Schema

The system uses Neon Postgres for storing conversation history:

- `chat_sessions` - Stores session metadata
- `chat_messages` - Stores individual messages with sources

## 🚀 Deployment

### Backend Deployment

The backend can be deployed to any platform that supports Python applications:

- **Railway**: `railway up`
- **Render**: Use the Python buildpack
- **AWS/Azure/GCP**: Deploy as a container or serverless function

### Frontend Deployment

The Docusaurus site can be deployed to:

- **GitHub Pages**: Built into Docusaurus
- **Vercel/Netlify**: Static site hosting
- **AWS S3/CloudFront**: Static hosting

## 🔒 Security Considerations

- Store API keys in environment variables, never in code
- Use HTTPS in production
- Implement rate limiting for API endpoints
- Validate user inputs

## 🤝 Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests if applicable
5. Submit a pull request

## 🐛 Troubleshooting

### Common Issues

1. **Backend not starting**: Ensure all environment variables are set
2. **Chat widget not appearing**: Check that the Layout override is working
3. **API errors**: Verify API keys and network connectivity
4. **Qdrant connection**: Ensure the cluster URL and API key are correct

### Debugging Tips

- Check the browser console for frontend errors
- Check the backend logs for API errors
- Verify the Qdrant collection exists and has data
- Ensure the OpenAI API key has sufficient quota

## 📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

## 🙏 Acknowledgments

- OpenAI for the GPT models and embeddings
- Qdrant for vector database technology
- Neon for serverless Postgres
- Docusaurus team for the documentation framework
- All contributors to the Humanoid Robotics Book project

---

For support, please open an issue in the repository or contact the maintainers.