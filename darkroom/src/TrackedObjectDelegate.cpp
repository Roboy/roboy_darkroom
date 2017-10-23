#include "darkroom/TrackedObjectDelegate.hpp"


void populateTableWidget(QTableWidget *tableWidget) {
    static const struct {
        const char *title;
        const char *genre;
        const char *artist;
        int rating;
    } staticData[] = {
            {"Mass in B-Minor",  "Baroque", "J.S. Bach",        5},
            {"Three More Foxes", "Jazz",    "Maynard Ferguson", 4},
            {"Sex Bomb",         "Pop",     "Tom Jones",        3},
            {"Barbie Girl",      "Pop",     "Aqua",             5},
            {0,                  0,         0,                  0}
    };

    for (int row = 0; staticData[row].title != 0; ++row) {
        QTableWidgetItem *item0 = new QTableWidgetItem(staticData[row].title);
        QTableWidgetItem *item1 = new QTableWidgetItem(staticData[row].genre);
        QTableWidgetItem *item2 = new QTableWidgetItem(staticData[row].artist);
        QTableWidgetItem *item3 = new QTableWidgetItem;
        item3->setData(0,
                       QVariant::fromValue(StarRating(staticData[row].rating)));

        tableWidget->setItem(row, 0, item0);
        tableWidget->setItem(row, 1, item1);
        tableWidget->setItem(row, 2, item2);
        tableWidget->setItem(row, 3, item3);
    }
}

void TrackedObjectDelegate::paint(QPainter *painter, const QStyleOptionViewItem &option,
                         const QModelIndex &index) const
{
    if (index.data().canConvert<StarRating>()) {
        StarRating starRating = qvariant_cast<StarRating>(index.data());

        if (option.state & QStyle::State_Selected)
            painter->fillRect(option.rect, option.palette.highlight());

        starRating.paint(painter, option.rect, option.palette,
                         StarRating::ReadOnly);
    } else {
        QStyledItemDelegate::paint(painter, option, index);
    }
}

QSize TrackedObjectDelegate::sizeHint(const QStyleOptionViewItem &option,
                             const QModelIndex &index) const
{
    if (index.data().canConvert<StarRating>()) {
        StarRating starRating = qvariant_cast<StarRating>(index.data());
        return starRating.sizeHint();
    } else {
        return QStyledItemDelegate::sizeHint(option, index);
    }
}

QWidget *TrackedObjectDelegate::createEditor(QWidget *parent,
                                    const QStyleOptionViewItem &option,
                                    const QModelIndex &index) const

{
    if (index.data().canConvert<StarRating>()) {
        TrackedObjectEditor *editor = new TrackedObjectEditor(parent);
        connect(editor, &TrackedObjectEditor::editingFinished,
                this, &TrackedObjectDelegate::commitAndCloseEditor);
        return editor;
    } else {
        return QStyledItemDelegate::createEditor(parent, option, index);
    }
}

void TrackedObjectDelegate::setEditorData(QWidget *editor,
                                 const QModelIndex &index) const
{
    if (index.data().canConvert<StarRating>()) {
        StarRating starRating = qvariant_cast<StarRating>(index.data());
        TrackedObjectEditor *trackedObjectEditor = qobject_cast<TrackedObjectEditor *>(editor);
        trackedObjectEditor->setStarRating(starRating);
    } else {
        QStyledItemDelegate::setEditorData(editor, index);
    }
}

void TrackedObjectDelegate::setModelData(QWidget *editor, QAbstractItemModel *model,
                                const QModelIndex &index) const
{
    if (index.data().canConvert<StarRating>()) {
        TrackedObjectEditor *trackedObjectEditor = qobject_cast<TrackedObjectEditor *>(editor);
        model->setData(index, QVariant::fromValue(trackedObjectEditor->starRating()));
    } else {
        QStyledItemDelegate::setModelData(editor, model, index);
    }
}

void TrackedObjectDelegate::commitAndCloseEditor()
{
    TrackedObjectEditor *editor = qobject_cast<TrackedObjectEditor *>(sender());
    emit commitData(editor);
    emit closeEditor(editor);
}

TrackedObjectEditor::TrackedObjectEditor(QWidget *parent)
        : QWidget(parent)
{
    setMouseTracking(true);
    setAutoFillBackground(true);
}

QSize TrackedObjectEditor::sizeHint() const
{
    return myStarRating.sizeHint();
}

void TrackedObjectEditor::paintEvent(QPaintEvent *)
{
    QPainter painter(this);
    myStarRating.paint(&painter, rect(), this->palette(),
                       StarRating::Editable);
}

void TrackedObjectEditor::mouseMoveEvent(QMouseEvent *event)
{
    int star = starAtPosition(event->x());

    if (star != myStarRating.starCount() && star != -1) {
        myStarRating.setStarCount(star);
        update();
    }
}

void TrackedObjectEditor::mouseReleaseEvent(QMouseEvent * /* event */)
{
    emit editingFinished();
}

int TrackedObjectEditor::starAtPosition(int x)
{
    int star = (x / (myStarRating.sizeHint().width()
                     / myStarRating.maxStarCount())) + 1;
    if (star <= 0 || star > myStarRating.maxStarCount())
        return -1;

    return star;
}

const int PaintingScaleFactor = 20;

StarRating::StarRating(int starCount, int maxStarCount)
{
    myStarCount = starCount;
    myMaxStarCount = maxStarCount;

    starPolygon << QPointF(1.0, 0.5);
    for (int i = 1; i < 5; ++i)
        starPolygon << QPointF(0.5 + 0.5 * std::cos(0.8 * i * 3.14),
                               0.5 + 0.5 * std::sin(0.8 * i * 3.14));

    diamondPolygon << QPointF(0.4, 0.5) << QPointF(0.5, 0.4)
                   << QPointF(0.6, 0.5) << QPointF(0.5, 0.6)
                   << QPointF(0.4, 0.5);
}

QSize StarRating::sizeHint() const
{
    return PaintingScaleFactor * QSize(myMaxStarCount, 1);
}

void StarRating::paint(QPainter *painter, const QRect &rect,
                       const QPalette &palette, EditMode mode) const
{
    painter->save();

    painter->setRenderHint(QPainter::Antialiasing, true);
    painter->setPen(Qt::NoPen);

    if (mode == Editable) {
        painter->setBrush(palette.highlight());
    } else {
        painter->setBrush(palette.foreground());
    }

    int yOffset = (rect.height() - PaintingScaleFactor) / 2;
    painter->translate(rect.x(), rect.y() + yOffset);
    painter->scale(PaintingScaleFactor, PaintingScaleFactor);

    for (int i = 0; i < myMaxStarCount; ++i) {
        if (i < myStarCount) {
            painter->drawPolygon(starPolygon, Qt::WindingFill);
        } else if (mode == Editable) {
            painter->drawPolygon(diamondPolygon, Qt::WindingFill);
        }
        painter->translate(1.0, 0.0);
    }

    painter->restore();
}