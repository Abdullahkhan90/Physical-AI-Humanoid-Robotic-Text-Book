from fastapi import FastAPI
from .api.modules import router as modules_router
from .api.content import router as content_router
from .api.chatbot import router as chatbot_router
from .api.preferences import router as preferences_router
from .api.learning_path import router as learning_path_router

app = FastAPI(title="AI Textbook API", version="1.0.0")

# Include routers
app.include_router(modules_router)
app.include_router(content_router)
app.include_router(chatbot_router)
app.include_router(preferences_router)
app.include_router(learning_path_router)

@app.get("/")
def read_root():
    return {"message": "AI Textbook API is running"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}