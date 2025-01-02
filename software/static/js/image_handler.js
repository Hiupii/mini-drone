// DOM Elements
const historyImages = document.querySelectorAll('.history-image');
const currentImage = document.getElementById('current-image');
const imageTimestamp = document.getElementById('image-timestamp');
const detailTimestamp = document.getElementById('detail-timestamp');

// Kiểm tra trạng thái ban đầu
function checkInitialState() {
  const noImageMessage = "Chọn ảnh"; // Thông báo khi chưa có ảnh
  if (!currentImage.src || currentImage.src.includes("placeholder")) {
    currentImage.style.display = "none"; // Ẩn khung ảnh
    imageTimestamp.textContent = noImageMessage; // Hiển thị thông báo
  }
}

// Gọi kiểm tra trạng thái ban đầu khi trang tải
checkInitialState();

// Xử lý việc chọn ảnh từ lịch sử
historyImages.forEach(image => {
  image.addEventListener('click', function () {
    // Cập nhật ảnh hiện tại
    currentImage.src = this.src;
    currentImage.style.display = "block"; // Hiển thị ảnh
    imageTimestamp.textContent = `Thời gian chụp: ${this.getAttribute('data-timestamp')}`;
    detailTimestamp.textContent = this.getAttribute('data-timestamp');
  });
});

function checkInitialState() {
  const noImageMessage = "Chọn ảnh";
  currentImage.style.display = "none"; // Ẩn khung ảnh
  imageTimestamp.textContent = noImageMessage; // Hiển thị thông báo
  imageTimestamp.style.fontSize = "24px"; // Tăng kích thước chữ (nếu muốn set inline)
  viewDetailsButton.style.display = "none"; // Ẩn nút "Xem chi tiết"
}

fetch('/get-images')
  .then(response => response.json())
  .then(images => {
    const historyContainer = document.getElementById('history-container');
    
    // Xóa ảnh cũ trong container (nếu có)
    historyContainer.innerHTML = '';
    
    // Duyệt qua danh sách ảnh và tạo liên kết cho mỗi ảnh
    images.forEach(image => {
      const link = document.createElement('a');
      link.href = `#`;
      link.classList.add('history-image-link');
      link.textContent = image;  // Hiển thị tên tệp ảnh
      link.addEventListener('click', function() {
        // Khi người dùng nhấn vào, cập nhật ảnh ở ô xem ảnh
        const currentImage = document.getElementById('current-image');
        const imageTimestamp = document.getElementById('image-timestamp');
        const detailTimestamp = document.getElementById('detail-timestamp');
        
        // Cập nhật ảnh
        currentImage.src = `/static/resources/${image}`;
        imageTimestamp.textContent = `Thời gian chụp: ${new Date().toISOString()}`;
        detailTimestamp.textContent = new Date().toISOString();
      });
      
      // Thêm liên kết vào container
      historyContainer.appendChild(link);
      historyContainer.appendChild(document.createElement('br'));  // Thêm xuống dòng
    });
  })
  .catch(error => console.error('Error loading images:', error));