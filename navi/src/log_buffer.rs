use std::collections::VecDeque;
use std::fmt;
use std::sync::Mutex;

use serde::Serialize;
use tracing::field::{Field, Visit};
use tracing::Subscriber;
use tracing_subscriber::layer::Context;
use tracing_subscriber::Layer;

const MAX_ENTRIES: usize = 500;

#[derive(Debug, Clone, Serialize)]
pub struct LogEntry {
    pub timestamp_ms: u64,
    pub level: String,
    pub target: String,
    pub message: String,
}

#[derive(Clone)]
pub struct LogBuffer {
    entries: std::sync::Arc<Mutex<VecDeque<LogEntry>>>,
    start: std::time::Instant,
}

impl LogBuffer {
    pub fn new() -> Self {
        Self {
            entries: std::sync::Arc::new(Mutex::new(VecDeque::with_capacity(MAX_ENTRIES))),
            start: std::time::Instant::now(),
        }
    }

    pub fn push(&self, entry: LogEntry) {
        let mut entries = self.entries.lock().unwrap();
        if entries.len() >= MAX_ENTRIES {
            entries.pop_front();
        }
        entries.push_back(entry);
    }

    pub fn recent(&self, limit: usize) -> Vec<LogEntry> {
        let entries = self.entries.lock().unwrap();
        let skip = entries.len().saturating_sub(limit);
        entries.iter().skip(skip).cloned().collect()
    }

    pub fn elapsed_ms(&self) -> u64 {
        self.start.elapsed().as_millis() as u64
    }
}

struct MessageVisitor {
    message: String,
    fields: Vec<String>,
}

impl Visit for MessageVisitor {
    fn record_debug(&mut self, field: &Field, value: &dyn fmt::Debug) {
        if field.name() == "message" {
            self.message = format!("{:?}", value);
        } else {
            self.fields.push(format!("{}={:?}", field.name(), value));
        }
    }
}

impl<S: Subscriber> Layer<S> for LogBuffer {
    fn on_event(&self, event: &tracing::Event<'_>, _ctx: Context<'_, S>) {
        let mut visitor = MessageVisitor {
            message: String::new(),
            fields: Vec::new(),
        };
        event.record(&mut visitor);

        let mut msg = visitor.message;
        if !visitor.fields.is_empty() {
            msg.push_str(" ");
            msg.push_str(&visitor.fields.join(" "));
        }

        self.push(LogEntry {
            timestamp_ms: self.elapsed_ms(),
            level: event.metadata().level().to_string(),
            target: event.metadata().target().to_string(),
            message: msg,
        });
    }
}
